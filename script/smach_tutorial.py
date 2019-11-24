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
        smach.State.__init__(self, outcomes=['to_Na','to_Pr','success'])
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
            return 'to_Pr'
        else:
            return 'success'


# define state Bar
class N(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa','to_Em'])

    def execute(self, userdata):
        req = PatrolCommandRequest()
        pat_ser = rospy.ServiceProxy('/patrol_server', PatrolCommand)

        x = rospy.get_param("goal_x")
        y = rospy.get_param("goal_y")
        name = rospy.get_param("goal_name")

        req.goal_position.position.x = x
        req.goal_position.position.y = y
        req.point.data = name

        result = pat_ser(req)

        if result.result.data == True:
            return 'to_Wa'
        else:
            return 'to_Em'

class P(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa'])

    def execute(self, userdata):
        x = rospy.get_param("goal_x")
        y = rospy.get_param("goal_y")
        return 'to_Wa'

class E(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa','to_Em'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(1)
        return 'to_Wa'



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Waypoint', W(),
                               transitions={'to_Na':'Navigation',
                                            'to_Pr':'Projection',
                                            'success':'success'})
        smach.StateMachine.add('Navigation', N(),
                               transitions={'to_Wa':'Waypoint',
                                            'to_Em':'Emergency'})

        smach.StateMachine.add('Projection', P(),
                               transitions={'to_Wa':'Waypoint',
                                            'to_Em':'Emergency'})

        smach.StateMachine.add('Emergency', E(),
                               transitions={'to_Wa':'Waypoint'})
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
