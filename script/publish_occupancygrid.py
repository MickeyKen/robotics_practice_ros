#!/usr/bin/python
import rospy

import numpy as np

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int8

def pub():

    rospy.init_node('occ_pub_node', anonymous=True)

    pub = rospy.Publisher('my_map', OccupancyGrid, queue_size=100)

    r = rospy.Rate(5)

    msg = OccupancyGrid()

    msg.header.frame_id = '/map'
    msg.header.stamp = rospy.Time.now()

    msg.info.resolution = 0.05
    msg.info.width = 30
    msg.info.height = 30

    msg.info.origin.orientation.w = 1.0

    data = np.array(range(msg.info.width * msg.info.height))

    r = rospy.Rate(5)

    while not rospy.is_shutdown():

        for i in range(msg.info.width * msg.info.height):
            data[i] = 70

        msg.data = data

        pub.publish(msg)
        r.sleep()


if __name__ == '__main__':
    try:
            pub()

    except rospy.ROSInterruptException: pass
