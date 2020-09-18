#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String

# define state Controller
class Controller(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'],
                                    input_keys=['cont_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Controller')
        selector = int(userdata.cont_input)
        if selector == 1:
            print('outcome1 from Controller')
            return 'outcome1'
        elif selector == 2:
            print('outcome2 from Controller')
            return 'outcome2'
        else:
            print('outcome3 from Controller')
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
        ud.mon_data = msg.data
        return False

    # main
    def main(self):
        rospy.init_node('smach_example_state_machine')

        # Create a SMACH state machine
        self.sm_top = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

        # Open the container
        with self.sm_top:
            # Add states to the container
            smach.StateMachine.add('Mon', smach_ros.MonitorState(
                "/sm_reset", String, self.monitor_cb, output_keys=['mon_data']),
                transitions={'invalid':'Controller', 'valid':'Controller', 'preempted':'Controller'},
                remapping={'mon_data':'sm_data'})
            
            smach.StateMachine.add('Controller', Controller(), 
                                transitions={'outcome1':'Controller', 
                                                'outcome2':'Second',
                                                'outcome3':'Mon'},
                                remapping={'cont_input':'sm_data'})

            smach.StateMachine.add('First', First(), 
                                transitions={'outcome2':'outcome4'})

            smach.StateMachine.add('Second', Second(), 
                                transitions={'outcome2':'outcome4'})

        # Execute SMACH plan
        outcome = self.sm_top.execute()


if __name__ == '__main__':
    sm = StateMachineController()
    sm.main()