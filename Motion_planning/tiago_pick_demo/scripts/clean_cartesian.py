#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
import numpy as np
import math


def clean_table():
    rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
    moveit_commander.roscpp_initialize([])
    group_arm_torso = MoveGroupCommander("arm_torso")
    group_arm_torso.set_planner_id("LIN")
    group_arm_torso.set_pose_reference_frame("base_footprint")
    group_arm_torso.set_max_velocity_scaling_factor(1.0)
    group_arm_torso.set_planning_time(10.0)  # Increase planning time

    ## These are the maximum specifications of the table for which the arm can clean in one go. ##
    sponge_width = 0.05
    table_width = 0.5
    table_depth = 0.3
    table_center_y = 0.0
    table_center_x = 0.6     # red axis
    table_center_z = 1.0
    step_num = 4
    # step_num = math.floor(table_width/(sponge_width))  # Decrease number of waypoints
    
    waypoints = []
    waypoints_y = np.linspace(table_center_y-table_width/2+sponge_width/2, table_center_y+table_width/2-sponge_width/2, step_num)
    waypoints_x = [table_center_x-table_depth/2+sponge_width/2, table_center_x+table_depth/2-sponge_width/2-0.05]
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
    rospy.loginfo("state: %s" % group_arm_torso.get_current_state())
    rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('clean_table', anonymous=True)
    clean_table()
    rospy.loginfo("Cleaning table finished")