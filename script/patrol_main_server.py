#!/usr/bin/env python

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from ubiquitous_display_msgs.srv import PatrolCommand
from ubiquitous_display_msgs.srv import PatrolCommandResponse


class Publishers():

    def make_goal_pub(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0
        goal_msg.header.frame_id = "map"
        self.goal_pub.publish(goal_msg)

    def make_frag_pub(self, x, y):
       markerArray = MarkerArray()

       marker = Marker()
       marker.header.frame_id = "/map"
       marker.type = marker.SPHERE
       marker.action = marker.ADD
       marker.scale.x = 1.0
       marker.scale.y = 1.0
       marker.scale.z = 0.0
       marker.color.a = 1.0
       marker.color.r = 1.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.pose.orientation.w = 1.0
       marker.pose.position.x = x
       marker.pose.position.y = y
       marker.pose.position.z = 3.0
       # marker.lifetime = -1

       markerArray.markers.append(marker)
       self.goal_frag_pub.publish(markerArray)


class Subscribe(Publishers):
    def __init__(self):
        self.exit = 0

        self.goal_frag_pub = rospy.Publisher('/goal_frag', MarkerArray, queue_size=100)

        # Declaration Publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

        # Declaration Subscriber
        # self.nav_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.nav_callback)
        self.nav_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav2_callback)
        # Declaration Service Server
        self.server = rospy.Service("/patrol_server", PatrolCommand, self.service_callback)

    ### callback function for amcl node (pose) ###
    # def nav_callback(self, msg):
    #     if msg.status_list:
    #         status_id = msg.status_list[0].status
    #
    #         if status_id == 0 or status_id == 3:
    #             self.exit = 1
    #             print "go"
    #         else:
    #             self.exit = 0
    #             print "wait"
    #     else:
    #         self.exit = 0
    def nav2_callback(self, msg):
        if msg.status:
            status_id = msg.status.status

            if status_id == 3:
                self.exit = 1
                print "go"
            else:
                self.exit = 0
                print "wait"
        else:
            self.exit = 0

    def service_callback(self, req):
        x = float(req.goal_position.position.x)
        y = float(req.goal_position.position.y)
        self.make_frag_pub(x, y)
        self.make_goal_pub(x,y)

        self.exit = 0

        print "Navigation .. "
        while True:
            # print self.exit
            if self.exit == 1:
                break
        print ("finish: navigation {0:4.0f}(m) {1:4.0f}(m)".format(x,y))

        result = Bool()
        result.data = True
        return PatrolCommandResponse(result)

if __name__ == '__main__':
    rospy.init_node('patrol_main_service_server')

    Subscribe = Subscribe()

    rospy.spin()
