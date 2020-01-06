#!/usr/bin/env python

import rospy
import numpy as np
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float64

import tf
from geometry_msgs.msg import Quaternion, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from ubiquitous_display_msgs.srv import *

class Publishers():

    def euler_to_quaternion(self, euler):
        """Convert Euler Angles to Quaternion

        euler: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion
        """
        q = tf.transformations.quaternion_from_euler(euler.x, euler.y, euler.z)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


    def make_pt_sv(self, pan,tilt):

        req = PantiltCommandRequest()
        req.pan_speed.data = 0.3
        req.tilt_speed.data = 0.3
        req.pan_degree.data = pan
        req.tilt_degree.data = tilt
        res = self.pt_sv_cl(req)



class Subscribe(Publishers):
    def __init__(self):

        # self.jsk_text_pub = rospy.Publisher('/goal_name', Pictogram, queue_size = 100)

        # Declaration Subscriber
        # self.nav_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.nav_callback)
        self.nav_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        # Declaration Service Server
        self.main_server = rospy.Service("/projection_server", PatrolCommand, self.main_service_callback)

        self.pt_sv_cl = rospy.ServiceProxy("/pantilt_radian_server", PantiltCommand)

        self.x_robot = 0.0
        self.y_robot = 0.0

    def amcl_callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y

    def main_service_callback(self, req):
        proj_x = float(req.goal_position.position.x)
        proj_y = float(req.goal_position.position.y)

        rospy.set_param("/head_trace_server/flag", "none")

        self.exit = 0


        result = Bool()
        result.data = True
        return PatrolCommandResponse(result)

if __name__ == '__main__':
    rospy.init_node('projection_service_server')

    Subscribe = Subscribe()

    rospy.spin()
