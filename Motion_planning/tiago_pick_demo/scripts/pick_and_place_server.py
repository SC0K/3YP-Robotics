#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Author:
#   * Sitong Chen
import sys
import rospy
from spherical_grasps_server import SphericalGrasps
from actionlib import SimpleActionClient, SimpleActionServer
import moveit_commander
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.msg import Grasp, PickupAction, PickupGoal, PickupResult, MoveItErrorCodes
from moveit_msgs.msg import PlaceAction, PlaceGoal, PlaceResult, PlaceLocation
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Vector3Stamped, Vector3, Quaternion
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal, PickUpPoseResult, PickUpPoseFeedback
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest, GetPlanningSceneResponse
from std_srvs.srv import Empty, EmptyRequest
from copy import deepcopy
from random import shuffle
import copy
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
	if not name[:1] == '_':
		code = MoveItErrorCodes.__dict__[name]
		moveit_error_dict[code] = name


def createPickupGoal(group="arm_torso", target="part",
					 grasp_pose=PoseStamped(),
					 possible_grasps=[],
					 links_to_allow_contact=None):
	""" Create a PickupGoal with the provided data"""
	pug = PickupGoal()
	pug.target_name = target
	pug.group_name = group
	pug.possible_grasps.extend(possible_grasps)
	pug.allowed_planning_time = 35.0
	pug.planning_options.planning_scene_diff.is_diff = True
	pug.planning_options.planning_scene_diff.robot_state.is_diff = True
	pug.planning_options.plan_only = False
	pug.planning_options.replan = True
	pug.planning_options.replan_attempts = 1  # 10
	pug.allowed_touch_objects = []
	pug.attached_object_touch_links = ['<octomap>']
	pug.attached_object_touch_links.extend(links_to_allow_contact)

	return pug


def createPlaceGoal(place_pose,
					place_locations,
					group="arm_torso",
					target="part",
					links_to_allow_contact=None):
	"""Create PlaceGoal with the provided data"""
	placeg = PlaceGoal()
	placeg.group_name = group
	placeg.attached_object_name = target
	placeg.place_locations = place_locations
	placeg.allowed_planning_time = 15.0
	placeg.planning_options.planning_scene_diff.is_diff = True
	placeg.planning_options.planning_scene_diff.robot_state.is_diff = True
	placeg.planning_options.plan_only = False
	placeg.planning_options.replan = True
	placeg.planning_options.replan_attempts = 1
	placeg.allowed_touch_objects = ['<octomap>']
	placeg.allowed_touch_objects.extend(links_to_allow_contact)

	return placeg

class PickAndPlaceServer(object):
	def __init__(self):
		rospy.loginfo("Initalizing PickAndPlaceServer...")
		self.sg = SphericalGrasps()
		rospy.loginfo("Connecting to pickup AS")
		self.pickup_ac = SimpleActionClient('/pickup', PickupAction)	# Create an action client for the '/pickup' action server -> line 213
		self.pickup_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		rospy.loginfo("Connecting to place AS")
		self.place_ac = SimpleActionClient('/place', PlaceAction)
		self.place_ac.wait_for_server()
		rospy.loginfo("Succesfully connected.")
		self.scene = PlanningSceneInterface()
		rospy.loginfo("Connecting to /get_planning_scene service")
		self.scene_srv = rospy.ServiceProxy(
			'/get_planning_scene', GetPlanningScene)
		self.scene_srv.wait_for_service()
		rospy.loginfo("Connected.")
		rospy.loginfo("Connecting to clear octomap service...")
		self.clear_octomap_srv = rospy.ServiceProxy(
			'/clear_octomap', Empty)
		self.clear_octomap_srv.wait_for_service()
		rospy.loginfo("Connected!")

		# Get the object size
		self.object_height = rospy.get_param('~object_height')
		self.object_width = rospy.get_param('~object_width')
		self.object_depth = rospy.get_param('~object_depth')

		# Get the links of the end effector exclude from collisions
		self.links_to_allow_contact = rospy.get_param('~links_to_allow_contact', None)
		if self.links_to_allow_contact is None:
			rospy.logwarn("Didn't find any links to allow contacts... at param ~links_to_allow_contact")
		else:
			rospy.loginfo("Found links to allow contacts: " + str(self.links_to_allow_contact))
#############################################################################
		self.set_table_as = SimpleActionServer(
			'/set_table', PickUpPoseAction,
			execute_cb=self.set_table_cb, auto_start=False)
		self.set_table_as.start()

		self.pick_as = SimpleActionServer(
			'/pickup_pose', PickUpPoseAction,
			execute_cb=self.pick_cb, auto_start=False)	# A callback function to execute when the action is called. In this case, the callback is self.pick_cb.
		self.pick_as.start()		# This starts the action server and allows it to begin accepting goals.

		self.clean_as = SimpleActionServer(
			'/clean_pose', PickUpPoseAction,
			execute_cb=self.clean_cb, auto_start=False)	# A callback function to execute when the action is called. In this case, the callback is self.pick_cb.
		self.clean_as.start()


		# self.clean_table_as = SimpleActionServer('clean_motion', PickUpPoseAction, self.clean_table_cb, False)
		# self.clean_table_as.start()
	
		self.place_as = SimpleActionServer(
			'/place_pose', PickUpPoseAction,
			execute_cb=self.place_cb, auto_start=False)
		self.place_as.start()	
#############################################################################
	# Not used at the moment
	def clean_table_cb(self,goal):
		rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
		moveit_commander.roscpp_initialize([])
		group_arm_torso = MoveGroupCommander("arm_torso")
		group_arm_torso.set_planner_id("SBLkConfigDefault")
		group_arm_torso.set_pose_reference_frame("base_footprint")
		group_arm_torso.set_max_velocity_scaling_factor(1.0)
		group_arm_torso.set_planning_time(20.0)  # Increase planning time
		table_pose = goal.object_pose
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
		table_width = 0.4
		table_depth = 0.3
		table_center_y = 0.0
		table_center_x = 0.7 # 0.7
		table_center_z = 1.0 #table_pose.pose.position.z + 0.4	
		step_num = math.floor(table_width/sponge_width) #step_num = math.floor(table_width/sponge_width+1)
		
		waypoints = []
		waypoints_y = np.linspace(table_center_y-table_width/2+sponge_width/2, table_center_y+table_width/2-sponge_width/2, step_num)
		waypoints_x = [table_center_x-table_depth/2+sponge_width/2, table_center_x+table_depth/2-sponge_width/2-0.05]
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

		result = PickUpPoseResult()
		if fraction == 1.0:
			rospy.loginfo('\033[92m' + "Plan found"+ '\033[0m')
		else:
			rospy.loginfo("Planning failed")
			result.error_code = 0
			self.clean_table_as.set_aborted(result)

		start = rospy.Time.now()
		# group_arm_torso.set_goal_position_tolerance(0.5)  # 5 cm
		# group_arm_torso.set_goal_orientation_tolerance(1.5)  # 0.5 radians
		group_arm_torso.execute(plan, wait=True)
		moveit_commander.roscpp_shutdown()
		result.error_code = 1
		rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())
		self.clean_table_as.set_succeeded(result)

	def clean_cb(self, goal):

		if goal is not None:
			self.table_pose = copy.deepcopy(goal.object_pose)    # copy goal from the client
			self.table_pose = copy.deepcopy(goal.object_pose)
			# rospy.loginfo("Removing any previous 'part' object")
			# self.scene.remove_attached_object("arm_tool_link")
			# self.scene.remove_world_object("part")
			self.scene.remove_world_object("table")
			rospy.loginfo("Clearing octomap")
			# self.clear_octomap_srv.call(EmptyRequest())
			# rospy.sleep(2.0)  # Removing is fast
			set_table_pose = copy.deepcopy(self.object_pose)

			#define a virtual table below the object
			# table_height = object_pose.pose.position.z - 0.016 - self.object_height/2 + 0.005
			table_height = self.table_pose.pose.position.z 
			table_width  = self.table_pose.pose.orientation.y
			table_depth  = self.table_pose.pose.orientation.x
			set_table_pose.pose.position.x = self.table_pose.pose.position.x
			set_table_pose.pose.position.y = self.table_pose.pose.position.y
			set_table_pose.pose.position.z = table_height/2
			rospy.loginfo("Table pose: %s", set_table_pose)

			self.scene.add_box("table", set_table_pose, (table_depth, table_width, table_height))		# What does this do? ############

			# # We need to wait for the object part to appear
			self.wait_for_planning_scene_object("table")
			error_code = 1
			p_res = PickUpPoseResult()
			rospy.loginfo("result: %s", p_res)
			p_res.error_code = error_code
			if error_code != 1:
				self.clean_as.set_aborted(p_res)
			else:
				self.clean_as.set_succeeded(p_res)
			rospy.loginfo('\033[92m' + "Table pose set successfully" + '\033[0m')
			rospy.loginfo("Table pose: %s", self.table_pose)
		else:
			rospy.logwarn('\033[91m' + "Received goal is None" + '\033[0m')

	# Return the error code
	
	def set_table_cb(self, goal):      
		"""
		:type goal: PickUpPoseGoal
		"""
		# Only accept the new goal if it's not None
		if goal is not None:
			self.table_pose = copy.deepcopy(goal.object_pose)    # copy goal from the client
			error_code = 1
			p_res = PickUpPoseResult()
			p_res.error_code = error_code
			if error_code != 1:
				self.set_table_as.set_aborted(p_res)
			else:
				self.set_table_as.set_succeeded(p_res)
			rospy.loginfo('\033[92m' + "Table pose set successfully by callback" + '\033[0m')
			rospy.loginfo("Table pose: %s", self.table_pose)
		else:
			rospy.logwarn('\033[91m' + "Received goal is None" + '\033[0m')


	def pick_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.grasp_object(goal.object_pose)
		self.object_pose = copy.deepcopy(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.pick_as.set_aborted(p_res)
		else:
			self.pick_as.set_succeeded(p_res)	# Acrion server sends a result to the client

	def place_cb(self, goal):
		"""
		:type goal: PickUpPoseGoal
		"""
		error_code = self.place_object(goal.object_pose)
		p_res = PickUpPoseResult()
		p_res.error_code = error_code
		if error_code != 1:
			self.place_as.set_aborted(p_res)
		else:
			self.place_as.set_succeeded(p_res)

	def wait_for_planning_scene_object(self, object_name='part'):
		rospy.loginfo(
			"Waiting for object '" + object_name + "'' to appear in planning scene...")
		gps_req = GetPlanningSceneRequest()
		gps_req.components.components = gps_req.components.WORLD_OBJECT_NAMES
		
		part_in_scene = False
		while not rospy.is_shutdown() and not part_in_scene:
			# This call takes a while when rgbd sensor is set
			gps_resp = self.scene_srv.call(gps_req)
			# check if 'part' is in the answer
			for collision_obj in gps_resp.scene.world.collision_objects:
				if collision_obj.id == object_name:
					part_in_scene = True
					break
			else:
				rospy.sleep(1.0)

		rospy.loginfo("'" + object_name + "'' is in scene!")

	def grasp_object(self, object_pose):
		rospy.loginfo("Removing any previous 'part' object")
		self.scene.remove_attached_object("arm_tool_link")
		self.scene.remove_world_object("part")
		self.scene.remove_world_object("table")
		rospy.loginfo("Clearing octomap")
		self.clear_octomap_srv.call(EmptyRequest())
		rospy.sleep(2.0)  # Removing is fast
		rospy.loginfo("Adding new 'part' object")

		rospy.loginfo("Object pose: %s", object_pose.pose)
		object_pose.pose.position.z += 0.016
		
		#Add object description in scene
		self.scene.add_box("part", object_pose, (self.object_depth, self.object_width, self.object_height))

		rospy.loginfo("Second%s", object_pose.pose)
		set_table_pose = copy.deepcopy(object_pose)

		#define a virtual table below the object
		# table_height = object_pose.pose.position.z - 0.016 - self.object_height/2 + 0.005
		table_height = self.table_pose.pose.position.z+0.025
		table_width  = self.table_pose.pose.orientation.y
		table_depth  = self.table_pose.pose.orientation.x
		set_table_pose.pose.position.x = self.table_pose.pose.position.x
		set_table_pose.pose.position.y = self.table_pose.pose.position.y
		set_table_pose.pose.position.z = table_height/2

		self.scene.add_box("table", set_table_pose, (table_depth, table_width, table_height))		# What does this do? ############

		# # We need to wait for the object part to appear
		self.wait_for_planning_scene_object()
		self.wait_for_planning_scene_object("table")

		# compute grasps
		possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
		self.pickup_ac	
		goal = createPickupGoal(
			"arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)

		rospy.loginfo("Sending goal")
		self.pickup_ac.send_goal(goal)			# Send the goal to the action server (/pickup) and wait for the result, but where is the server?
		rospy.loginfo("Waiting for result")
		self.pickup_ac.wait_for_result()		
		result = self.pickup_ac.get_result()
		rospy.logdebug("Using torso result: " + str(result))
		rospy.loginfo(
			"Pick result: " +
		str(moveit_error_dict[result.error_code.val]))

		return result.error_code.val	# Return the error code

		

	def place_object(self, object_pose):
		rospy.loginfo("Clearing octomap")		## why clean octomap?
		self.clear_octomap_srv.call(EmptyRequest())
		possible_placings = self.sg.create_placings_from_object_pose(
			object_pose)
		# Try only with arm
		rospy.loginfo("Trying to place using only arm")
		goal = createPlaceGoal(
			object_pose, possible_placings, "arm", "part", self.links_to_allow_contact)
		rospy.loginfo("Sending goal")
		self.place_ac.send_goal(goal)
		rospy.loginfo("Waiting for result")

		self.place_ac.wait_for_result()
		result = self.place_ac.get_result()
		rospy.loginfo(str(moveit_error_dict[result.error_code.val]))

		if str(moveit_error_dict[result.error_code.val]) != "SUCCESS":
			rospy.loginfo(
				"Trying to place with arm and torso")
			# Try with arm and torso
			goal = createPlaceGoal(
				object_pose, possible_placings, "arm_torso", "part", self.links_to_allow_contact)
			rospy.loginfo("Sending goal")
			self.place_ac.send_goal(goal)
			rospy.loginfo("Waiting for result")

			self.place_ac.wait_for_result()
			result = self.place_ac.get_result()
			rospy.logerr(str(moveit_error_dict[result.error_code.val]))
		
		# print result
		rospy.loginfo(
			"Result: " +
			str(moveit_error_dict[result.error_code.val]))
		rospy.loginfo("Removing previous 'part' object")
		self.scene.remove_world_object("part")

		return result.error_code.val


if __name__ == '__main__':
	rospy.init_node('pick_and_place_server')
	paps = PickAndPlaceServer()
	rospy.spin()
