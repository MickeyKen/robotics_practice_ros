#!/usr/bin/env python

import rospy
import smach
import smach_ros

from smach_ros import ServiceState, SimpleActionState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import String, Int16
from ubiquitous_display_msgs.srv import *

# define state Foo
class W(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Na','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        request = rospy.ServiceProxy('/patrol_waypoint', SmachCommand)
        req = Int16()
        req.data = self.counter
        responce = request(req)

        self.counter += 1

        if responce.result.data == "navigation":
            return 'to_Na'
        elif responce.result.data == "projection":
            return 'to_Na'
        else:
            return 'outcome2'


# define state Bar
class N(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(1)
        return 'to_Wa'




# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Waypoint', W(),
                               transitions={'to_Na':'Navigation',
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('Navigation', N(),
                               transitions={'to_Wa':'Waypoint'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
