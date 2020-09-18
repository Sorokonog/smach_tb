#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String

# define state Controller
class Controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3']
                                            , input_keys=['controller_input'])
        self.class_selector = 0
        print(self.class_selector)

    def execute(self, userdata):
        rospy.loginfo('Executing state Controller')
        if self.class_selector == 1:
            return 'outcome1'
        elif self.class_selector == 2:
            return 'outcome2'
        else:
            return 'outcome3'


# define state Bar
class First(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state First')
        return 'outcome2'


# define state Bar
class Second(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Second')
        return 'outcome2'
        

class StateMachineController():

    def __init__(self):
        pass

    def monitor_cb(self, ud, msg):
        Controller.class_selector = int(msg.data)
        return False

    # main
    def main(self):
        rospy.init_node('smach_example_state_machine')

        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('Controller', Controller(), 
                                transitions={'outcome1':'Controller', 
                                                'outcome2':'Second',
                                                'outcome3':'Mon'})

            smach.StateMachine.add('Mon', smach_ros.MonitorState(
                "/sm_reset", String, self.monitor_cb), 
                transitions={'invalid':'Controller', 'valid':'Controller', 'preempted':'Controller'}, output_keys=['mon_output'])

            smach.StateMachine.add('First', First(), 
                                transitions={'outcome2':'outcome4'})

            smach.StateMachine.add('Second', Second(), 
                                transitions={'outcome2':'outcome4'})

        # Execute SMACH plan
        outcome = sm.execute()


if __name__ == '__main__':
    sm = StateMachineController()
    sm.main()