#!/usr/bin/env python

import rospy
import numpy as np
from math import radians, copysign, sqrt, pow, pi, atan2, degrees
import tf
import PyKDL

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion

MAX_PAN_POS = 2.7
MIN_PAN_POS = -2.7

MAX_TILT_POS = 0.30
MIN_TILT_POS = -0.34

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkPanLimitPosition(pos):
    pos = constrain(pos, MIN_PAN_POS, MAX_PAN_POS)

    return pos

def checkTiltLimitPosition(pos):
    pos = constrain(pos, MIN_TILT_POS, MAX_TILT_POS)

    return pos


def head_make_gazebo(pan, tilt):
    pan_pub = rospy.Publisher('/ubiquitous_display/pan_controller/command', Float64, queue_size=10)
    tilt_pub = rospy.Publisher('/ubiquitous_display/tilt_controller/command', Float64, queue_size=10)
    pan_pub.publish(pan-1.57)
    tilt_pub.publish(tilt)

def get_distance(x, y):
    d = sqrt((x) ** 2 + (y) ** 2)
    return d


def cal(x, y, z):
    rad90 = radians(90)
    rad30 = radians(30)
    pan = 0.0
    tilt = 0.0
    theta = atan2(y, x)
    pan = theta

    distance = get_distance(x, y)

    # print z - 0.673
    # print distance

    tilt = atan2(z - 0.673, distance)
    # print degrees(tilt)
    tilt = (tilt - rad30)


    pan = checkPanLimitPosition(pan)
    tilt = checkTiltLimitPosition(tilt)

    return pan, -tilt

if __name__ == '__main__':
    rospy.init_node('head_trace_server')

    rospy.set_param("/head_trace_server/flag", "none")

    rate = rospy.Rate(10.0)

    listener = tf.TransformListener()


    while not rospy.is_shutdown():
        flag = rospy.get_param("/head_trace_server/flag")

        if flag == "on":
            try:
                (trans,rot) = listener.lookupTransform('/base_link', "/target", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            pan, tilt = cal(trans[0], trans[1], trans[2])
            # print pan, tilt
            head_make_gazebo(pan, 0)

        elif flag == "none":
            pass

        else:
            pass

        rate.sleep()
