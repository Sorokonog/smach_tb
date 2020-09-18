#!/usr/bin/env python

import rospy
import smach

class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        rospy.loginfo('Shutting down')
        return 'exit'

class Pause(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['main_controller_state'])

    def execute(self, userdata):
        rospy.loginfo('Pausing robot')
        #DO SOME STUFF HERE
        return 'main_controller_state'

class NextGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['main_controller_state'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Shutdown')
        return 'main_controller_state'

class MainControllerState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome_exit'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Main loop')
        #DO SOME MAIN CONTROLLER HERE
        return 'outcome_exit' 

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['close_sm', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Shutdown', Shutdown(), 
                               transitions={'exit':'close_sm'})
        smach.StateMachine.add('NextGoal', NextGoal(), 
                               transitions={'outcome_exit':'Shutdown'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()