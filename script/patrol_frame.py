#!/usr/bin/env python

import rospy
import numpy as np
import math
import os
import rospkg

from ubiquitous_display_msgs.srv import PatrolCommand, PatrolCommandRequest
from ubiquitous_display_msgs.srv import PatrolCommandResponse

LOOP_NUM = 2
POINT_NUM = 3
LOOP_NUM *= POINT_NUM

rospy.init_node('patrol_main_server')

pat_ser = rospy.ServiceProxy('/patrol_server', PatrolCommand)

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

for loop in range(LOOP_NUM):
    x = float(goal_pos[loop % POINT_NUM][0])
    y = float(goal_pos[loop % POINT_NUM][1])
    name = str(name[loop % POINT_NUM])
    req = PatrolCommandRequest()
    req.goal_position.position.x = x
    req.goal_position.position.y = y
    req.point.data = name
    result = pat_ser(req)
    result = result.result.data
    print x, y, result
