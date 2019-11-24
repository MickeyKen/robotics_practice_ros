#!/usr/bin/env python

import rospy, rospkg
import numpy as np
import math
import os

from std_msgs.msg import String, Int16

from ubiquitous_display_msgs.srv import SmachCommand, SmachCommandRequest
from ubiquitous_display_msgs.srv import SmachCommandResponse

import tf
from geometry_msgs.msg import Quaternion, Vector3

LOOP_NUM = 2
POINT_NUM = 3
LOOP_NUM *= POINT_NUM

class Publishers():

    def euler_to_quaternion(self, euler):
        pass


class Subscribe(Publishers):
    def __init__(self):
        self.goal_pos = np.zeros((POINT_NUM, 2), dtype = 'float64')
        self.proj_pos = np.zeros((POINT_NUM, 2), dtype = 'float64')
        self.name = []
        count = 0

        file_name = "goal_points"
        path = os.path.join(rospkg.RosPack().get_path('robotics_practice')+ '/txt', file_name)
        filepath = os.path.splitext(path)[0] + '.txt'
        file_content = open(filepath)

        for texts in file_content:
            text = texts.replace('\n'," ")
            text = texts.split(" ")
            self.goal_pos[count][0] = float(text[0])
            self.goal_pos[count][1] = float(text[1])
            self.name.append(str(text[2]))
            self.proj_pos[count][0] = float(text[3])
            self.proj_pos[count][1] = float(text[4])

            count += 1
        # Declaration Service Server
        self.server = rospy.Service("/patrol_waypoint", SmachCommand, self.service_callback)


    def service_callback(self, req):
        count = req.count.data
        result = String()

        if count > LOOP_NUM:
            result.data = "False"
            return SmachCommandResponse(result)
        else:
            if (count % 2) == 0:
                index = count / 2
                x = float(self.goal_pos[index][0])
                y = float(self.goal_pos[index][1])
                point_name = name[index]
                rospy.set_param('goal_x', x)
                rospy.set_param('goal_y', y)
                rospy.set_param('goal_name', point_name)
                result.data = "navigation"
                return SmachCommandResponse(result)
            else:
                index = (count - 1) / 2
                x = float(self.proj_pos[index][0])
                y = float(self.proj_pos[index][1])
                rospy.set_param('proj_x', x)
                rospy.set_param('proj_y', y)
                result.data = "projection"
                return SmachCommandResponse(result)


if __name__ == '__main__':
    rospy.init_node('waypoint_server')

    Subscribe = Subscribe()

    rospy.spin()
