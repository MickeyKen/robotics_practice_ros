#!/usr/bin/env python

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import Pictogram

from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from ubiquitous_display_msgs.srv import PatrolCommand
from ubiquitous_display_msgs.srv import PatrolCommandResponse

import tf
from geometry_msgs.msg import Quaternion, Vector3

class Publishers():

    def euler_to_quaternion(self, euler):
        """Convert Euler Angles to Quaternion

        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def make_jsk_text_pub(self, name, x, y):
        qua = Quaternion()
        qua = self.euler_to_quaternion(Vector3(0.0, -1.57, 0.0))
        print qua
        jsk_text_msg = Pictogram()
        jsk_text_msg.header.stamp = rospy.Time.now()
        jsk_text_msg.header.frame_id = "map"
        jsk_text_msg.pose.position.x = x
        jsk_text_msg.pose.position.y = y
        jsk_text_msg.pose.position.z = 1.5
        jsk_text_msg.pose.orientation.x = qua.x
        jsk_text_msg.pose.orientation.y = qua.y
        jsk_text_msg.pose.orientation.z = qua.z
        jsk_text_msg.pose.orientation.w = qua.w
        jsk_text_msg.character = name
        jsk_text_msg.mode = 1
        jsk_text_msg.color.b = 1.0
        jsk_text_msg.color.g = 1.0
        jsk_text_msg.color.a = 1.0
        jsk_text_msg.action = 4
        jsk_text_msg.speed = 0.3
        self.jsk_text_pub.publish(jsk_text_msg)

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
        marker.id = 1
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        # marker.lifetime = -1
        markerArray.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 2
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.55
        # marker.lifetime = -1
        markerArray.markers.append(marker)

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = 3
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.215
        # marker.lifetime = -1
        markerArray.markers.append(marker)
        self.goal_frag_pub.publish(markerArray)

    def make_pan(self, name):

        pan_msg = String()
        pan_msg.data = "pan"
        self.pan_pub.publish(pan_msg)


class Subscribe(Publishers):
    def __init__(self):
        self.exit = 0

        self.jsk_text_pub = rospy.Publisher('/goal_name', Pictogram, queue_size = 100)

        self.goal_frag_pub = rospy.Publisher('/goal_frag', MarkerArray, queue_size=100)

        self.pan_pub = rospy.Publisher('/navigation/pan', String, queue_size=10)

        # Declaration Publisher
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)

        # Declaration Subscriber
        # self.nav_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.nav_callback)
        self.nav_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.nav2_callback)
        # Declaration Service Server
        self.server = rospy.Service("/patrol_server", PatrolCommand, self.service_callback)

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
        text = req.point.data
        print text
        self.make_jsk_text_pub(text, x, y)
        self.make_frag_pub(x, y)
        self.make_goal_pub(x,y)

        self.exit = 0

        print "Navigation .. "
        while True:
            self.make_pan("pan")
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
