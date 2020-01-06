#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.srv import GetMap

import matplotlib.pyplot as plt
import math


def plot(x, y):

    plt.plot(x, y, "o")
    plt.axis("equal")
    # plt.draw()
    # plt.pause(0.00000000001)

def main():

    rospy.init_node('view_map_service_data')

    map_service = rospy.ServiceProxy('static_map', GetMap)

    result = map_service()

    # for plotting
    plt.ion()
    plt.show()

    ### get width and height of Maps ###
    width = result.map.info.width
    height = result.map.info.height

    # initialize array for plotting
    plot_x = []
    plot_y = []

    ### create x-y pose for plotting
    for i in range(len(result.map.data)):
        occ = result.map.data[i]
        # print occ
        if (occ == 100):
            plot_x.append(i / width)
            plot_y.append(i % width)

    # print len(plot_x)
    # print len(plot_y)

    plot(plot_x, plot_y)
    plt.show(block = True)

if __name__ == '__main__':
    main()
