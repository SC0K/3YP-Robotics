#!/usr/bin/env python

# Author:
#   * Sitong Chen

import rospy
import time
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, CleaningAction, CleaningActionResult, TableAction, TableActionResult
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty
import math

import cv2
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name

class SphericalService(object):
	def __init__(self):
		rospy.loginfo("Starting Spherical Grab Service")
		self.pick_type = PickAruco()			# Create an object of the class PickAruco, initialize the object, creating two action clients: '/place_pose' and '/pickup_pose'. Set the publishers to the torso and head controller				
		rospy.loginfo("Finished SphericalService constructor")
		self.place_gui = rospy.Service("/place_gui", Empty, self.start_aruco_place)		# Reduntant. There is no operation for "place" in the pick_aruco function (see below) 
		# self.pick_gui = rospy.Service("/pick_gui", Empty, self.start_aruco_pick)		# pick, clean, and place.
		self.pick_as = SimpleActionServer('/picking_action', CleaningAction, execute_cb=self.motion_task, auto_start=False)
		self.pick_as.start()
		self.clean_as = SimpleActionServer('/cleaning_action', CleaningAction, execute_cb=self.motion_task, auto_start=False)
		self.clean_as.start()
		self.place_as = SimpleActionServer('/placing_action', CleaningAction, execute_cb=self.motion_task, auto_start=False)
		self.place_as.start()

#================================================================================================================================================================
	def motion_task(self, goal):
		if goal.task.data == "pick":
			result = self.pick_type.pick_aruco("pick")
			self.pick_as.set_succeeded(result.result) 
		elif goal.task.data == "clean":
			result = self.pick_type.clean_table_master("clean")
			self.clean_as.set_succeeded(result.result)   
		else:
			result = self.pick_type.place()
			self.place_as.set_succeeded(result.result)
		
		return {}
#================================================================================================================================================================
	
	
	def start_aruco_place(self, req):
		self.pick_type.pick_aruco("place") 		# Redundant. There is no operation for "place" in the pick_aruco function
		return {}

class PickAruco(object):
	def __init__(self):
		rospy.loginfo("Initalizing...")
		self.bridge = CvBridge()
		self.tfBuffer = tf2_ros.Buffer()
		self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
		rospy.loginfo("Waiting for /pickup_pose AS...")
		self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction) 	# Create an action client for the '/pickup_pose' action server
		time.sleep(1.0)
		if not self.pick_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /pickup_pose AS")
			exit()

		self.set_table_as = SimpleActionClient('/set_table', PickUpPoseAction) 	# Create an action client for the '/clean_table' action server
		self.set_table_as.wait_for_server()
		rospy.loginfo('\033[92m' + "Connected to set_table server!" + '\033[0m')

		self.clean_ac = SimpleActionClient('/clean_pose', PickUpPoseAction) 	# Create an action client for the '/table_pose' action server
		self.clean_ac.wait_for_server()
		rospy.loginfo('\033[92m' + "Connected to table_pose server!" + '\033[0m')

		rospy.loginfo("Waiting for /place_pose AS...")
		self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction) 

		self.place_as.wait_for_server()

		rospy.loginfo("Setting publishers to torso and head controller...")
		self.torso_cmd = rospy.Publisher(
			'/torso_controller/command', JointTrajectory, queue_size=1)
		self.head_cmd = rospy.Publisher(
			'/head_controller/command', JointTrajectory, queue_size=1)
		self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
							 PoseStamped,
							 queue_size=1,
							 latch=True)

		rospy.loginfo("Waiting for '/play_motion' AS...")
		self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)	# Create an action client for the '/play_motion' action server
		if not self.play_m_as.wait_for_server(rospy.Duration(20)):
			rospy.logerr("Could not connect to /play_motion AS")
			exit()
		rospy.loginfo("Connected!")
		rospy.sleep(1.0)
		rospy.loginfo("Done initializing PickAruco.")

		self.tableID_as = SimpleActionServer('/tableID_action', TableAction, execute_cb=self.set_tableID, auto_start=False)
		self.tableID_as.start()

	def set_tableID(self, goal):
		self.table_id = goal.table_id
		rospy.loginfo("Table ID is set to: " + str(self.table_id))
		result = TableActionResult()
		result.result.error_code = 1
		self.tableID_as.set_succeeded(result.result)
		return {}

	def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s
	
	def place(self):

#================================================================
#The following code is for placing the object back to its original position.
#================================================================	
		# Raise arm
		self.prepare_placing_robot()
		rospy.loginfo("Raise object done.")

		# Place the object back to its position
		rospy.loginfo("Gonna place near where it was")
		self.pick_g.object_pose.pose.position.z += 0.07
		self.place_as.send_goal_and_wait(self.pick_g)
		rospy.loginfo("Done!")
		result = CleaningActionResult()
		result.result.error_code = 1
		return result

	def clean_table_master(self, string_operation):
		self.prepare_clean()
		if string_operation == "clean":
			#======================================================================== Getting The table pose from Apriltag ==================================================
			
			detector = AprilTagDetector()
			detector.Set_tableID(self.table_id)
			while not detector.table_detected:
				rospy.loginfo("Waiting for table detection")
				rospy.sleep(1.0)
			table_info_goal = PickUpPoseGoal()
			table_info_goal.object_pose.pose.orientation.y = detector.tag_info[1]
			table_info_goal.object_pose.pose.orientation.x = detector.tag_info[2]
			table_info_goal.object_pose.pose.position.x = detector.tag_pose_relative_to_base_stamped.pose.position.x
			table_info_goal.object_pose.pose.position.y = detector.tag_pose_relative_to_base_stamped.pose.position.y
			table_info_goal.object_pose.pose.position.z = detector.tag_pose_relative_to_base_stamped.pose.position.z

			table_info_goal.object_pose.header.frame_id = 'base_footprint'
			table_info_goal.object_pose.pose.orientation.w = 1.0
			rospy.loginfo('\033[92m' + str(table_info_goal) + '\033[0m')
			self.set_table_as.send_goal_and_wait(table_info_goal)
			rospy.loginfo('\033[92m' + "Done setting table!" + '\033[0m')
			self.clean_ac.send_goal_and_wait(table_info_goal)			# Send the goal to the action server (/pickup_pose) and wait for the result
			rospy.loginfo("Done setting cleaning scene!")
			result = self.clean_table(table_info_goal.object_pose)		# Clean the table

			rospy.loginfo("Cleaning Done")
			return result


	
#==============================================================================================================================================================
		
	def pick_aruco(self, string_operation):
		self.prepare_robot()		# Lower head.


#=============================================================== Getting the pose of the aruco marker ===============================================================
		rospy.sleep(2.0)
		rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection")

		aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)		# Getting the pose of the aruco marker
		aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
		rospy.loginfo("Got: " + str(aruco_pose))


		rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
		aruco_pose.header.frame_id + " to 'base_footprint'")
		ps = PoseStamped()
		ps.pose.position = aruco_pose.pose.position
		ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
		ps.header.frame_id = aruco_pose.header.frame_id
		transform_ok = False
		while not transform_ok and not rospy.is_shutdown():
			try:
				transform = self.tfBuffer.lookup_transform("base_footprint", 
									   ps.header.frame_id,
									   rospy.Time(0))
				aruco_ps = do_transform_pose(ps, transform)
				transform_ok = True
			except tf2_ros.ExtrapolationException as e:
				rospy.logwarn(
					"Exception on transforming point... trying again \n(" +
					str(e) + ")")
				rospy.sleep(0.01)
				ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
			self.pick_g = PickUpPoseGoal()		# The Goal message for the action server
#==============================================================================================================================================================
		#======================================================================== Getting The table pose from Apriltag ==================================================
			detector = AprilTagDetector()
			detector.Set_tableID(self.table_id)
			while not detector.table_detected:
				rospy.loginfo("Waiting for table detection")
				rospy.sleep(1.0)
			table_info_goal = PickUpPoseGoal()
			table_info_goal.object_pose.pose.orientation.y = detector.tag_info[1]
			table_info_goal.object_pose.pose.orientation.x = detector.tag_info[2]
			table_info_goal.object_pose.pose.position.x = detector.tag_pose_relative_to_base_stamped.pose.position.x
			table_info_goal.object_pose.pose.position.y = detector.tag_pose_relative_to_base_stamped.pose.position.y
			table_info_goal.object_pose.pose.position.z = detector.tag_pose_relative_to_base_stamped.pose.position.z

			table_info_goal.object_pose.header.frame_id = 'base_footprint'
			table_info_goal.object_pose.pose.orientation.w = 1.0
			rospy.loginfo('\033[92m' + str(table_info_goal) + '\033[0m')
			self.set_table_as.send_goal_and_wait(table_info_goal)
			rospy.loginfo("Done Picking the object now")
#==============================================================================================================================================================

		if string_operation == "pick":

			rospy.loginfo("Setting cube pose based on ArUco detection")
			self.pick_g.object_pose.pose.position = aruco_ps.pose.position
			self.pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)-0.04			# The marker is placed at the top of the marker, so we need to move the gripper to the center of the object
			rospy.loginfo("aruco pose in base_footprint:" + str(self.pick_g))

			self.pick_g.object_pose.header.frame_id = 'base_footprint'
			self.pick_g.object_pose.pose.orientation.w = 1.0
			self.detected_pose_pub.publish(self.pick_g.object_pose)		# Publish the detected pose of the aruco marker to the controller and start the pick 
			rospy.loginfo("Gonna pick:" + str(self.pick_g))
			self.pick_as.send_goal_and_wait(self.pick_g)			# Send the goal to the action server (/pickup_pose) and wait for the result
			rospy.loginfo("Done!")

			result = self.pick_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")
				return
			result = CleaningActionResult()
			result.result.error_code = 1
			return result
			

	def lower_head(self):
		rospy.loginfo("Moving head down")
		jt = JointTrajectory()
		jt.joint_names = ['head_1_joint', 'head_2_joint']
		jtp = JointTrajectoryPoint()
		jtp.positions = [0.0, -0.75]
		jtp.time_from_start = rospy.Duration(2.0)
		jt.points.append(jtp)
		self.head_cmd.publish(jt)
		rospy.loginfo("Done.")

	def prepare_robot(self):
		rospy.loginfo("Unfold arm safely")
		pmg = PlayMotionGoal()
		pmg.motion_name = 'pregrasp'
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Done.")

		self.lower_head()

		rospy.loginfo("Robot prepared.")
				
	def prepare_placing_robot(self):
		rospy.loginfo("Grasp Success")
		pmg = PlayMotionGoal()
		pmg.motion_name = 'pick_final_pose'         # Where is this defined? 
		pmg.skip_planning = False
		self.play_m_as.send_goal_and_wait(pmg)
		rospy.loginfo("Done.")

		self.lower_head()

		rospy.loginfo("Robot prepared to place")

	def clean_table(self, table_pose):
			rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
			moveit_commander.roscpp_initialize([])
			group_arm_torso = MoveGroupCommander("arm_torso")
			group_arm_torso.set_planner_id("LIN")
			group_arm_torso.set_pose_reference_frame("base_footprint")
			group_arm_torso.set_max_velocity_scaling_factor(1.0)
			group_arm_torso.set_planning_time(20.0)  # Increase planning time

			sponge_width = 0.05
			table_width = table_pose.pose.orientation.y-0.34
			table_depth = table_pose.pose.orientation.x-0.1
			table_center_y = table_pose.pose.position.y
			table_center_x = table_pose.pose.position.x
			table_center_z = table_pose.pose.position.z + 0.3 #object_pose.pose.position.z+0.05
			if table_width > 0.6:
				table_width = 0.6
			if table_depth > 0.4:
				table_depth = 0.4
			if table_center_x > 0.6:
				table_center_x = 0.6
			
			## Dont change these values for now
			table_width = 0.5
			table_depth = 0.3
			table_center_y = 0.0
			table_center_x = 0.7
			table_center_z = table_pose.pose.position.z + 0.15
			step_num = math.floor(table_width/sponge_width+1)
			
			waypoints = []
			waypoints_y = np.linspace(table_center_y-table_width/2+sponge_width/2, table_center_y+table_width/2-sponge_width/2, step_num)
			waypoints_x = [table_center_x-table_depth/2+sponge_width/2, table_center_x+table_depth/2-sponge_width/2]
			waypoints_z_angle = np.linspace(-50, 50, step_num)

			for i in range(step_num):
				for x in waypoints_x:
					waypoint = Pose()
					waypoint.position.x = x
					waypoint.position.y = waypoints_y[i]
					waypoint.position.z = table_center_z
					q = quaternion_from_euler(math.radians(-90), 0, math.radians(waypoints_z_angle[i]))
					waypoint.orientation.x = q[0]
					waypoint.orientation.y = q[1]
					waypoint.orientation.z = q[2]
					waypoint.orientation.w = q[3]
					waypoints.append(waypoint)

			(plan, fraction) = group_arm_torso.compute_cartesian_path(waypoints, 0.01, 0.0, path_constraints=None, avoid_collisions=True)  # Removed orientation_tolerance

			result = CleaningActionResult()
			if fraction == 1.0:
				rospy.loginfo('\033[92m' + "Plan found"+ '\033[0m')
			else:
				rospy.loginfo("Planning failed")
				result.result.error_code = 0
				return result

			start = rospy.Time.now()
			group_arm_torso.execute(plan, wait=True)
			rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())
			moveit_commander.roscpp_shutdown()
			
			result.result.error_code = 1
			return result

	def prepare_clean(self):
		# rospy.loginfo('\033[92m' + "Prepare Cleaning" + '\033[0m')
		moveit_commander.roscpp_initialize([])
		# if len(args) < 7:
		#     rospy.loginfo("Usage: plan_arm_torso_ik  x y z  r p y")
		#     rospy.loginfo("where the list of arguments specify the target pose of /arm_tool_link expressed in /base_footprint")
		#     return

		# Default values
		args = [0.52, -0.4, 1.15, -1.57, 0.0, 0.0] 
		

		goal_pose = PoseStamped()
		goal_pose.header.frame_id = "base_footprint"
		goal_pose.pose.position.x = float(args[0])
		goal_pose.pose.position.y = float(args[1])
		goal_pose.pose.position.z = float(args[2])
		q = quaternion_from_euler(float(args[3]), float(args[4]), float(args[5]))
		goal_pose.pose.orientation.x = q[0]
		goal_pose.pose.orientation.y = q[1]
		goal_pose.pose.orientation.z = q[2]
		goal_pose.pose.orientation.w = q[3]

		group_arm_torso = MoveGroupCommander("arm_torso")
		group_arm_torso.set_planner_id("LIN")
		group_arm_torso.set_pose_reference_frame("base_footprint")
		group_arm_torso.set_pose_target(goal_pose)

		rospy.loginfo("Planning to move %s to a target pose expressed in %s" % 
					(group_arm_torso.get_end_effector_link(), group_arm_torso.get_planning_frame()))

		group_arm_torso.set_start_state_to_current_state()
		group_arm_torso.set_max_velocity_scaling_factor(1.0)

		group_arm_torso.set_planning_time(10.0)
		success, plan, planning_time, _ = group_arm_torso.plan()

		if success:
			rospy.loginfo("Plan found in %s seconds" % planning_time)
			rospy.loginfo('\033[92m' + "Prepare cleaning Success!!!!!!!!!" + '\033[0m')
		else:
			rospy.loginfo("Planning failed")


		start = rospy.Time.now()
		# group_arm_torso.go()
		group_arm_torso.execute(plan, wait=True)
		rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())

		moveit_commander.roscpp_shutdown()

		self.lower_head()

				
		
class AprilTagDetector:
	def __init__(self):
		self.latest_msg = None
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.table_apriltag_s = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.process_info)
		self.table_detected = False
	def Set_tableID(self, table_id):
		self.table_id = table_id
		rospy.loginfo("Table ID is set to: " + str(self.table_id))	
	def process_info(self, msg):
		# tag id: [table_height, table_width, table_depth]
		id_to_info = {0: [0.6, 1.2, 0.6], 1: [.6, 1, 1]}

		for detection in msg.detections:
			tag_id = detection.id[0]	
			if tag_id == self.table_id:  # to change, to be sent by task planning
				self.table_apriltag_s.unregister()
				# rospy.loginfo(f"apriltag id: {tag_id}")
				self.tag_info = id_to_info[tag_id]
				# rospy.loginfo(f"height: {tag_info[0]}")
				rospy.loginfo(f"width: {self.tag_info[1]}")
				rospy.loginfo(f"depth: {self.tag_info[2]}")
				# to add info on height, width, depth

				tag_pose_relative_to_camera = detection.pose.pose.pose
				# transform to PoseStamped
				tag_pose_relative_to_camera_stamped = PoseStamped()
				tag_pose_relative_to_camera_stamped.header.stamp = rospy.Time.now()
				tag_pose_relative_to_camera_stamped.header.frame_id = "xtion_rgb_optical_frame"
				tag_pose_relative_to_camera_stamped.pose = tag_pose_relative_to_camera

				# rospy.loginfo(f"Pose relative to camera: {tag_pose_relative_to_camera}") # debugging

				try:
					transform = self.tf_buffer.lookup_transform("base_footprint", "xtion_rgb_optical_frame", rospy.Time(0), rospy.Duration(1.0))
					rospy.loginfo(f"transform: {transform}")
				except Exception as e:
					rospy.loginfo("failed to lookup transform")
					rospy.loginfo(e)

				self.tag_pose_relative_to_base_stamped = tf2_geometry_msgs.do_transform_pose(tag_pose_relative_to_camera_stamped, transform)
				# tag_pose_relative_to_base = self.tag_pose_relative_to_base_stamped.pose
				self.table_detected = True

				# rospy.loginfo(f"transformed pose: {tag_pose_relative_to_base}")



if __name__ == '__main__':
	rospy.init_node('pick_aruco_demo')
	sphere = SphericalService()
	rospy.spin()
