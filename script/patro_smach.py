#!/usr/bin/env python
import rospy, actionlib, math, tf, rospkg
import numpy as np
import smach
import smach_ros
import random
from smach_ros import ServiceState, SimpleActionState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from std_msgs.msg import String


class Waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pr', 'to_Na', 'success'])

    def waypoint(self, userdata):
        goal_pos = np.zeros((POINT_NUM, 2), dtype = 'float64')
        name = []

        file_name = "goal_points"
        path = os.path.join(rospkg.RosPack().get_path('robotics_practice')+ '/txt', file_name)
        filepath = os.path.splitext(path)[0] + '.txt'
        file_content = open(filepath)
        count = 0
        for texts in file_content:
            text = texts.replace('\n'," ")
            text = texts.split(" ")
            goal_pos[count][0] = float(text[0])
            goal_pos[count][1] = float(text[1])
            # name[count] = str(text[2])
            name.append(str(text[2]))
            count += 1
            
        if responce.result.data == "nohuman":
            return 'to_Pa'
        else:
            rospy.set_param('/target_human/name', responce.result.data)
            return 'success'

class Navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pa', 'success'])

    def navigation(self, userdata):
        pass

class Projection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pa', 'success'])

    def projection(self, userdata):
        pass

class Emergecy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_Pa', 'success'])

    def emergecy(self, userdata):
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
