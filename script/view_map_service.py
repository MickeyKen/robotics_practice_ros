#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.srv import GetMap



def main():

    rospy.init_node('view_map_service_data')

    map_service = rospy.ServiceProxy('static_map', GetMap)

    result = map_service()

    print result.map.info.width

if __name__ == '__main__':
    main()
