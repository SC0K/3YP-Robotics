import roslib
import rospy
import smach
import smach_ros
import actionlib

# need to import locations

class NavigateToSurface(smach.State):
    def __init__(self, target_location):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.target_location = target_location
        self.client = actionlib.SimpleActionClient('movebase', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Navigating to surface')
        goal = MoveBaseGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Successflly navigated to surface')
            return 'success'
        else:
            rospy.loginfo('Failed to navigate to the surface')
            return 'failure'
        


class CleanSurface(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.client = actionlib.SimpleActionClient('cleaning_action', CleaningAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        rospy.loginfo('Starting cleaning')
        goal = CleaningGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Surface cleaning successful')
            return 'success'
        else:
            rospy.loginfo('Surface cleaning successful')
            return 'failure'
        

class ReturnToBase(smach.State):
    def __init__(self, base_location):
        smach.State.__init__(self, outcomes['success', 'failure'])
        self.base_location = base_location
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server

    def execute(self, userdata):
        rospy.loginfo('Returning to base')
        goal = MoveBaseGoal()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('Successfully returned to base')
            return 'success'
        else:
            rospy.loginfo('Failed to return to base')
            return 'failure'
        

def main():
    rospy.init_node('robot_cleaning_task')

    sm = smach.StateMachine(outcomes=['task_completed', 'returned_to_base', 'task_failed'])


    base = [0, 0, 0]
    surface_1 = [1, 1, 1]
    surface_2 = [2, 2, 2]

    with sm:

        # navigate and clean first surface
        smach.StateMachine.add('NAVIGATETOSURFACE1', NavigateToSurface(target_location=surface_1),
                               transitions = {'success':'CLEANSURFACE1', 'failure':'RETURNTOBASE'})
        
        smach.StateMachine.add('CLEANSURFACE1', CleanSurface(),
                               transitions = {'success':'task_completed', 'failure':'RETURNTOBASE'})
        
        # navigate and clean second surface

        smach.StateMachine.add('NAVIGATETOSURFACE2', NavigateToSurface(target_location=surface_2),
                               transitions={'success':'CLEANSURFACE2', 'failure':'RETURNTOBASE'})
        
        smach.StateMachine.add('CLEANSURFACE2', CleanSurface(),
                               transitions={'success':'task_completed', 'failure':'RETURNTOBASE'})
        
        # return to base

        smach.StateMachine.add('RETURNTOBASE', ReturnToBase(base_location=base),
                               transitions={'success':'returned_to_base', 'failure':'task_failed'})

    outcome = sm.execute()



if __name__ == '__main__':
    main()