#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D

# for plot
import math
import numpy as np
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Subscribe():
    def __init__(self):

        self.subscriber = rospy.Subscriber('/map', OccupancyGrid, self.callback)
        self.map_data = np.zeros([300,300])
        self.width = 0
        self.height = 0

    def plot(self, x, y):

        plt.plot(x, y, "o")
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)


    def callback(self, msg):

        # print msg.data[0]
        self.width = msg.info.width
        self. height = msg.info.height

        plot_x = []
        plot_y = []
        # print plot_x
        # print plot_y


        for i in range(len(msg.data)):
            occ = msg.data[i]
            # print occ
            if (occ == 100):
                plot_x.append(i / self.width)
                plot_y.append(i % self.width)
        print len(plot_x)
        print len(plot_y)
        self.plot(plot_x, plot_y)


def main():

    rospy.init_node('view_map_data')

    sub = Subscribe()

    # for plotting
    plt.ion()
    plt.show()

    rospy.spin()

if __name__ == '__main__':
    main()
