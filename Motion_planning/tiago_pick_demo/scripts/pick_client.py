#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty
import math

import cv2
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler

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
		self.pick_gui = rospy.Service("/pick_gui", Empty, self.start_aruco_pick)		# pick, clean, and place.

	def start_aruco_pick(self, req):
		self.pick_type.pick_aruco("pick")		
		return {}

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

	def strip_leading_slash(self, s):
		return s[1:] if s.startswith("/") else s
		
	def pick_aruco(self, string_operation):

#=============================================================== Getting the pose of the aruco marker ===============================================================
		self.prepare_robot()		# Lower head.

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
			pick_g = PickUpPoseGoal()		# The Goal message for the action server
#==============================================================================================================================================================
		if string_operation == "pick":

			rospy.loginfo("Setting cube pose based on ArUco detection")
			pick_g.object_pose.pose.position = aruco_ps.pose.position
			pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)-0.04			# The marker is placed at the top of the marker, so we need to move the gripper to the center of the object
			rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))

			pick_g.object_pose.header.frame_id = 'base_footprint'
			pick_g.object_pose.pose.orientation.w = 1.0
			self.detected_pose_pub.publish(pick_g.object_pose)		# Publish the detected pose of the aruco marker to the controller and start the pick 
			rospy.loginfo("Gonna pick:" + str(pick_g))
			self.pick_as.send_goal_and_wait(pick_g)			# Send the goal to the action server (/pickup_pose) and wait for the result
			rospy.loginfo("Done!")

			result = self.pick_as.get_result()
			if str(moveit_error_dict[result.error_code]) != "SUCCESS":
				rospy.logerr("Failed to pick, not trying further")
				return
			
#================================================================
#The following code is for placing the object back to its original position.
#================================================================
			
			self.clean_table(pick_g.object_pose)		# Clean the table
			rospy.loginfo("Cleaning Done")

			# Raise arm
			self.prepare_placing_robot()
			rospy.loginfo("Raise object done.")

			# Place the object back to its position
			rospy.loginfo("Gonna place near where it was")
			pick_g.object_pose.pose.position.z += 0.07
			self.place_as.send_goal_and_wait(pick_g)
			rospy.loginfo("Done!")

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

	def clean_table(self, object_pose):
			rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
			moveit_commander.roscpp_initialize([])
			group_arm_torso = MoveGroupCommander("arm_torso")
			group_arm_torso.set_planner_id("LIN")
			group_arm_torso.set_pose_reference_frame("base_footprint")
			group_arm_torso.set_max_velocity_scaling_factor(1.0)
			group_arm_torso.set_planning_time(10.0)  # Increase planning time

			sponge_width = 0.05
			table_width = 0.5
			table_depth = 0.3
			table_center_y = object_pose.pose.position.y
			table_center_x = object_pose.pose.position.x
			table_center_z = object_pose.pose.position.z+0.05
			step_num = math.floor(table_width/sponge_width+1)
			
			waypoints = []
			waypoints_y = np.linspace(table_center_y-table_width/2+sponge_width/2, table_center_y+table_width/2-sponge_width/2, step_num)
			waypoints_x = [table_center_x-table_depth/2+sponge_width/2, table_center_x+table_depth/2-sponge_width/2]
			waypoints_z_angle = np.linspace(-45, 45, step_num)

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

			if fraction == 1.0:
				rospy.loginfo("Plan found")
			else:
				rospy.loginfo("Planning failed")

			start = rospy.Time.now()
			group_arm_torso.execute(plan, wait=True)
			rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())
			moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
	rospy.init_node('pick_aruco_demo')
	sphere = SphericalService()
	rospy.spin()
