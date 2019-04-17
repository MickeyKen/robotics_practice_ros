#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.srv import GetMap

import matplotlib.pyplot as plt
import math


class Subscribe():
    def __init__(self, map_width, map_height, map_data):

        self.width = map_width
        self.height = map_height
        self.map_data = map_data

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.callback)


    def plot(self, x, y):

        plt.plot(x, y, "o")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)


    def callback(self, msg):

        # initialize array for plotting
        plot_x = []
        plot_y = []

        for i in range(len(self.map_data)):
            
        self.plot(0.0,0.0)

        # for publisher
        self.make_msg(0.0,0.0,0.0)
        self.send_msg()

def main():

    rospy.init_node('moving_obstacle')

    map_service = rospy.ServiceProxy('static_map', GetMap)
    result = map_service()

    # for plotting
    plt.ion()
    plt.show()

    sub = Subscribe(result.map.info.width, result.map.info.height, result.map.data)

    rospy.spin()

if __name__ == '__main__':
    main()
