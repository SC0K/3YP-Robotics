#!/usr/bin/env python
# -*- coding: utf-8 *-*

import roslib
import rospy
import smach
import smach_ros
import actionlib
from tiago_pick_demo.msg import CleaningAction, CleaningGoal, TableAction, TableGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# need to import locations

# class NavigateToSurface(smach.State):
#     def __init__(self, target_location):
#         smach.State.__init__(self, outcomes=['success', 'failure'])
#         self.target_location = target_location
#         self.client = actionlib.SimpleActionClient('movebase', MoveBaseAction)
#         self.client.wait_for_server()

#     def execute(self, userdata):
#         rospy.loginfo('Navigating to surface')
#         goal = MoveBaseGoal()
#         self.client.send_goal(goal)
#         self.client.wait_for_result()
#         if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
#             rospy.loginfo('Successflly navigated to surface')
#             return 'success'
#         else:
#             rospy.loginfo('Failed to navigate to the surface')
#             return 'failure'
class NavigateToSurface(smach.State):
    def __init__(self, target_location):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target_location = target_location
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Navigating to surface')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.target_location[0]
        goal.target_pose.pose.position.y = self.target_location[1]
        goal.target_pose.pose.position.z = 0 
        goal.target_pose.pose.orientation.x = self.target_location[2]
        goal.target_pose.pose.orientation.y = self.target_location[3]
        goal.target_pose.pose.orientation.z = self.target_location[4]
        goal.target_pose.pose.orientation.w = self.target_location[5]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(self.client.get_goal_status_text())
        rospy.loginfo(f"goal status: {self.client.get_state()}")
        if self.client.get_state() == 3:
            rospy.loginfo('Successfully navigated to surface')
            return 'success'
        else:
            rospy.loginfo('Failed to navigate to the surface')
            return 'failure'

class NavigateToBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target_location = [-0.5, 0, 0, 0, 0, 1] # assumes spawns at base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Navigating to Base')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.target_location[0]
        goal.target_pose.pose.position.y = self.target_location[1]
        goal.target_pose.pose.position.z = 0 
        goal.target_pose.pose.orientation.x = self.target_location[2]
        goal.target_pose.pose.orientation.y = self.target_location[3]
        goal.target_pose.pose.orientation.z = self.target_location[4]
        goal.target_pose.pose.orientation.w = self.target_location[5]
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(self.client.get_goal_status_text())
        rospy.loginfo(f"goal status: {self.client.get_state()}")
        if self.client.get_state() == 3:
            rospy.loginfo('Successfully navigated to base')
            return 'success'
        else:
            rospy.loginfo('Failed to navigate to the base')
            return 'failure' 


class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.client = actionlib.SimpleActionClient('picking_action', CleaningAction)
        self.client.wait_for_server()

        
    def execute(self, userdata):
        rospy.loginfo('Starting picking')
        goal = CleaningGoal()  
        goal.task.data = "pick"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Picking successful')
            return 'success'
        else:
            rospy.loginfo('Picking unsuccessful')
            return 'failure'
        
class CleanSurface(smach.State):
    def __init__(self,timeout_seconds = 70):
        smach.State.__init__(self, outcomes=['success', 'failure','timeout'])
        self.client = actionlib.SimpleActionClient('cleaning_action', CleaningAction)
        self.client.wait_for_server()
        self.timeout_seconds = rospy.Duration(timeout_seconds)

        
    def execute(self, userdata):
        rospy.loginfo('Starting cleaning')
        goal = CleaningGoal()  
        goal.task.data = "clean"
        self.client.send_goal(goal)
        # self.client.wait_for_result()
        if self.client.wait_for_result(self.timeout_seconds):
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo('Surface cleaning successful')
                return 'success'
            else:
                rospy.loginfo('Surface cleaning unsuccessful')
                return 'failure'
        else:
            rospy.loginfo('Surface cleaning timed out')
            return 'timeout'
            
    #def shutdown(self):
    #	self.client.cancel_all_goals()
    #	self.client.shutdown()

class Place(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.client = actionlib.SimpleActionClient('placing_action', CleaningAction)
        self.client.wait_for_server()

        
    def execute(self, userdata):
        rospy.loginfo('Starting placing')
        goal = CleaningGoal()  
        goal.task.data = "place"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Placing successful')
            return 'success'
        else:
            rospy.loginfo('Place unsuccessful')
            return 'failure'
class TableID(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['table_id'])
        self.client = actionlib.SimpleActionClient('/tableID_action', TableAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Starting placing')
        goal = TableGoal()  
        goal.table_id = userdata.table_id
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Placing successful')
            return 'success'
        else:
            rospy.loginfo('Place unsuccessful')
            return 'failure'
                
        
# class ReturnToBase(smach.State):
#     def __init__(self, base_location):
#         smach.State.__init__(self, outcomes['success', 'failure'])
#         self.base_location = base_location
#         self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#         self.client.wait_for_server

#     def execute(self, userdata):
#         rospy.loginfo('Returning to base')
#         goal = MoveBaseGoal()
#         self.client.send_goal(goal)
#         self.client.wait_for_result()
#         if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
#             rospy.loginfo('Successfully returned to base')
#             return 'success'
#         else:
#             rospy.loginfo('Failed to return to base')
#             return 'failure'
        

def main():
    rospy.init_node('robot_cleaning_task')

    sm = smach.StateMachine(outcomes=['task_completed', 'returned_to_base', 'task_failed'])


    surface_1 = [-3.1, -1, 0, 0, 1, 0] # april tag id=1
    surface_2 = [-2.8, -1, 0, 0, 0 , 1] #april tag id=0
    surface_3 = [-3.1, 1, 0, 0, 1, 0] #april tag id=2
    surface_4 = [-2.8, 1, 0, 0, 0, 1] #april tag id=4
    sponge_table = [-0.136, 0.703, 0, 0, 0, 1] # april tag id=3

    with sm:

        # navigate and clean first surface
        # smach.StateMachine.add('NAVIGATETOSURFACE1', NavigateToSurface(target_location=surface_1),
        #                        transitions = {'success':'CLEANSURFACE1', 'failure':'RETURNTOBASE'})
        smach.StateMachine.add('NAVIGATETOSPONGE', NavigateToSurface(target_location=sponge_table),
                               transitions = {'success':'sponge', 'failure':'RETURNTOBASE'})
                                
        smach.StateMachine.add('sponge', TableID(),
                               transitions = {'success':'PICK', 'failure':'RETURNTOBASE'},
                               remapping={'table_id': 'sponge_id'})
        
        smach.StateMachine.add('PICK', Pick(),
                               transitions = {'success':'NAVIGATETOSURFACE1', 'failure':'RETURNTOBASE'})

        smach.StateMachine.add('NAVIGATETOSURFACE1', NavigateToSurface(target_location=surface_1),
                               transitions = {'success':'Table1', 'failure':'NAVIGATETOSURFACE2'})

        smach.StateMachine.add('Table1', TableID(),
                               transitions = {'success':'CLEANSURFACE1', 'failure':'NAVIGATETOSURFACE2'},
                               remapping={'table_id': 'table_id1'})
       
        smach.StateMachine.add('CLEANSURFACE1', CleanSurface(),
                               transitions = {'success':'NAVIGATETOSURFACE3', 'failure':'RETURNTOBASE', 'timeout' : 'NAVIGATETOSURFACE3'})
                               

        smach.StateMachine.add('NAVIGATETOSURFACE3', NavigateToSurface(target_location=surface_3),
                               transitions = {'success':'Table3', 'failure':'RETURNTOBASE'})
        			
        smach.StateMachine.add('Table3', TableID(),
                               transitions = {'success':'CLEANSURFACE3', 'failure':'RETURNTOBASE'},
                               remapping={'table_id': 'table_id3'})	
        			
        smach.StateMachine.add('CLEANSURFACE3', CleanSurface(),
                               transitions = {'success':'NAVIGATETOSURFACE4', 'failure':'RETURNTOBASE', 'timeout' : 'NAVIGATETOSURFACE4'})
                               
        smach.StateMachine.add('NAVIGATETOSURFACE4', NavigateToSurface(target_location=surface_4),
                               transitions = {'success':'Table4', 'failure':'RETURNTOBASE'})
        			
        smach.StateMachine.add('Table4', TableID(),
                               transitions = {'success':'CLEANSURFACE4', 'failure':'RETURNTOBASE'},
                               remapping={'table_id': 'table_id4'})	
                               
        smach.StateMachine.add('CLEANSURFACE4', CleanSurface(),
                               transitions = {'success':'NAVIGATETOSURFACE2', 'failure':'RETURNTOBASE', 'timeout' : 'NAVIGATETOSURFACE2'})

        smach.StateMachine.add('NAVIGATETOSURFACE2', NavigateToSurface(target_location=surface_2),
        			transitions = {'success':'Table2', 'failure':'RETURNTOBASE'})
        			
        smach.StateMachine.add('Table2', TableID(),
                               transitions = {'success':'CLEANSURFACE2', 'failure':'RETURNTOBASE'},
                               remapping={'table_id': 'table_id2'})
        			
        smach.StateMachine.add('CLEANSURFACE2', CleanSurface(),
                               transitions = {'success':'RETURNTOBASE', 'failure':'RETURNTOBASE', 'timeout' : 'RETURNTOBASE'})

                   
  
        smach.StateMachine.add('RETURNTOBASE', NavigateToBase(),
                               transitions={'success':'returned_to_base', 'failure':'task_failed'})        
                               
                               
                                      # smach.StateMachine.add('PICK', Pick(),
                           #    transitions = {'success':'CLEANSURFACE1', 'failure':'task_failed'})
        
        #smach.StateMachine.add('PLACE', Place(),
                            #   transitions = {'success':'task_completed', 'failure':'task_failed'})
    sm.userdata.sponge_id = 3
    sm.userdata.table_id1 = 1  
    sm.userdata.table_id2 = 0
    sm.userdata.table_id3 = 2  
    sm.userdata.table_id4 = 4
        # # navigate and clean second surface

        # smach.StateMachine.add('NAVIGATETOSURFACE2', NavigateToSurface(target_location=surface_2),
        #                        transitions={'success':'CLEANSURFACE2', 'failure':'RETURNTOBASE'})
        
        # smach.StateMachine.add('CLEANSURFACE2', CleanSurface(),
        #                        transitions={'success':'task_completed', 'failure':'RETURNTOBASE'})
        
        # # return to base

        # smach.StateMachine.add('RETURNTOBASE', ReturnToBase(base_location=base),
        #                        transitions={'success':'returned_to_base', 'failure':'task_failed'})

    sis = smach_ros.IntrospectionServer('tg', sm, '/Cleaning_State_Machine')
    sis.start()
    
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
