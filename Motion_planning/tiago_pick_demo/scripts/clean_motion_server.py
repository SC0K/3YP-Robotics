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
from actionlib import SimpleActionClient, SimpleActionServer
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseFeedback, PickUpPoseResult

class CleanMotion():
    def __init__(self):
        self.clean_table_as = SimpleActionServer('/clean_motion', PickUpPoseAction, self.clean_table, False)
        self.clean_table_as.start()
    def clean_table(self,goal):
                rospy.loginfo('\033[92m' + "Cleaning table" + '\033[0m')
                moveit_commander.roscpp_initialize([])
                group_arm_torso = MoveGroupCommander("arm_torso")
                group_arm_torso.set_planner_id("LIN") # SBLkConfigDefault
                group_arm_torso.set_pose_reference_frame("base_footprint")
                group_arm_torso.set_max_velocity_scaling_factor(1.0)
                group_arm_torso.set_planning_time(20.0)  # Increase planning time
                table_pose = goal.object_pose
                sponge_width = 0.1
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
                    rospy.loginfo('\033[92m' + "Plan found in the motion server"+ '\033[0m')
                else:
                    rospy.loginfo("Planning failed")
                    result.error_code = 0
                    self.clean_table_as.set_aborted(result)

                start = rospy.Time.now()
                # group_arm_torso.set_goal_position_tolerance(0.5)  # 5 cm
                # group_arm_torso.set_goal_orientation_tolerance(1.5)  # 0.5 radians
                group_arm_torso.execute(plan, wait=True)
                group_arm_torso.stop()
                rospy.loginfo("Motion duration: %s seconds" % (rospy.Time.now() - start).to_sec())
                moveit_commander.roscpp_shutdown()
                result.error_code = 1
                self.clean_table_as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('clean_table', anonymous=True)
    cm = CleanMotion()
    rospy.spin()