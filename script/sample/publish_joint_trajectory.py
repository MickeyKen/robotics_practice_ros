#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import String, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


rospy.init_node('dynamixel_workbench')

mypub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', JointTrajectory, queue_size=100)

jt_msg = JointTrajectory()
count = 0
while True:
    jt_msg = JointTrajectory()
    jt_msg.header.stamp = rospy.Time.now()
    jt_msg.joint_names = [ "pan_joint", "tilt_joint", "yaw_joint" ]

    # create a JTP instance and configure it
    for i in range(3):
        jtp_msg = JointTrajectoryPoint()
        jtp_msg.velocities = [0.1]
        # setup the rest of the pt
        jtp_msg.time_from_start = rospy.Duration.from_sec(60.1)

        jt_msg.points.append(jtp_msg)

    mypub.publish(jt_msg)
    count += 1
    if count > 1000000:
        break
