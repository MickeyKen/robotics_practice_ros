#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose2D

# for plot
import math
import numpy as np
import matplotlib.pyplot as plt

class Publishsers():
    def make_msg(self, x, y, theta):

       self.message.x = x
       self.message.y = y
       self.message.theta = theta

    def send_msg(self):
        self.publisher.publish(self.message)


class Subscribe_publishers(Publishsers):
    def __init__(self):

        self.subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)
        self.publisher = rospy.Publisher('/calcurated_pose', Pose2D, queue_size=10)
        self.message = Pose2D()


    def plot(self, x, y):

        plt.plot(x, y, "o")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)


    def callback(self, msg):

        self.plot(msg.pose.pose.position.x, msg.pose.pose.position.y)

        # for publisher
        self.make_msg(0.0,0.0,0.0)
        self.send_msg()

def main():

    rospy.init_node('view_amcl_pose')

    sub = Subscribe_publishers()

    # for plotting
    plt.ion()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    main()
