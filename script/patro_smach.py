#!/usr/bin/env python
import rospy, math, tf, rospkg
import numpy as np
import smach
import smach_ros
import random
from smach_ros import ServiceState, SimpleActionState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import String

from ubiquitous_display_msgs.srv import *

class Waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pr', 'to_Na', 'success'])
        self.count = 0

    def waypoint(self, userdata):

        request = rospy.ServiceProxy('/patrol_waypoint', SmachCommand)
        req = Int16()
        req.data = self.count
        responce = request(req)

        self.count += 1

        if responce.result.data == "navigation":
            return 'to_Na'
        elif responce.result.data == "projection":
            return 'to_Pr'
        else:
            return 'success'

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa', 'to_Em'])

    def navigation(self, userdata):
        sum = 0
        if sum == 0:
            return 'to_Wa'
        else:
            return 'to_Em'

class Projection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa', 'to_Em'])

    def projection(self, userdata):
        sum = 0
        if sum == 0:
            return 'to_Wa'
        else:
            return 'to_Em'

class Emergency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Wa'])

    def emergecy(self, userdata):
        sum = 0
        if sum == 0:
            return 'to_Wa'
        else:
            pass

if __name__ == '__main__':

    rospy.init_node('patrol_smach')

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        ###start Search_and_Wander ###
        pa_sub = smach.StateMachine(outcomes=['success'])
        with pa_sub:
            smach.StateMachine.add('Waypoint',Waypoint(),
                                transitions={'to_Na':'Navigation',
                                            'to_Pr': 'Projection',
                                            'success': 'success'})

            smach.StateMachine.add('Navigation', Navigation(),
                                transitions={'to_Wa':'Waypoint',
                                'to_Em':'Emergency'})

            smach.StateMachine.add('Projection', Projection(),
                                transitions={'to_Wa':'Waypoint',
                                'to_Em': 'Emergency'})

            smach.StateMachine.add('Emergency', Emergency(),
                                transitions={'to_Wa':'Waypoint'})

        smach.StateMachine.add('Patrol', pa_sub,
                                transitions={'success': 'success'})


    sis = smach_ros.IntrospectionServer('Patrol_state_machine', sm, '/ROOT')
    sis.start()
    rospy.sleep(1)

    result = sm.execute()
    rospy.loginfo('result: %s' % result)
    rospy.spin()
    sis.stop()
