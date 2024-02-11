#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import numpy as np
import math

def clean_table():
			rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
			moveit_commander.roscpp_initialize([])
			waypoints = []
			sponge_width = 0.05
			table_width = 0.6
			table_depth = 0.4
			table_center_y = 0.0
			table_center_x = 0.4+0.2
			table_center_z = 0.8+0.05
			step_num = math.floor(table_width/sponge_width+1)
			
			waypoints_y = np.linspace(table_center_y-table_width/2+sponge_width/2, table_center_y+table_width/2-sponge_width/2, step_num)
			waypoints_x = [table_center_x-table_depth/2+sponge_width/2, table_center_x+table_depth/2-sponge_width/2]
			
			# waypoints_x = [table_center_x-table_width/2+sponge_width/2, table_center_x+table_width/2-sponge_width/2, step_num]
			# waypoints_y = np.linspace(table_center_y-table_depth/2+sponge_width/2, table_center_y+table_depth/2-sponge_width/2)
			waypoints_z_angle = np.linspace(-45, 45, step_num)
			for i in range(step_num):
				waypoints_q = quaternion_from_euler(math.radians(-90), 0,math.radians(waypoints_z_angle[i]))
				waypoints.append([waypoints_x[0], waypoints_y[i], table_center_z, waypoints_q[0], waypoints_q[1], waypoints_q[2], waypoints_q[3]])
				waypoints.append([waypoints_x[1], waypoints_y[i], table_center_z, waypoints_q[0], waypoints_q[1], waypoints_q[2], waypoints_q[3]])
			for waypoint in waypoints:
				
				goal_pose = PoseStamped()
				goal_pose.header.frame_id = "base_footprint"
				goal_pose.pose.position.x = waypoint[0]
				goal_pose.pose.position.y = waypoint[1]
				goal_pose.pose.position.z = waypoint[2]
				goal_pose.pose.orientation.x = waypoint[3]
				goal_pose.pose.orientation.y = waypoint[4]
				goal_pose.pose.orientation.z = waypoint[5]
				goal_pose.pose.orientation.w = waypoint[6]

				group_arm_torso = MoveGroupCommander("arm_torso")
				group_arm_torso.set_planner_id("LIN")
				group_arm_torso.set_pose_reference_frame("base_footprint")
				group_arm_torso.set_pose_target(goal_pose)

				rospy.loginfo("Planning to move %s to a target pose expressed in %s" % 
							(group_arm_torso.get_end_effector_link(), group_arm_torso.get_planning_frame()))

				group_arm_torso.set_start_state_to_current_state()
				group_arm_torso.set_max_velocity_scaling_factor(1.0)

				group_arm_torso.set_planning_time(5.0)
				success, plan, planning_time, _ = group_arm_torso.plan()

				if success:
					rospy.loginfo("Plan found in %s seconds" % planning_time)
				else:
					rospy.loginfo("Planning failed")


				start = rospy.Time.now()
				group_arm_torso.go()
				rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())

			moveit_commander.roscpp_shutdown()
			

if __name__ == '__main__':
    rospy.init_node('clean_table', anonymous=True)
    clean_table()
    rospy.loginfo("Cleaning table finished")
	