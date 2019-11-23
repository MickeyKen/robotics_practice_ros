#!/usr/bin/env python

import rospy
import numpy as np
from math import radians, copysign, sqrt, pow, pi
import tf
import PyKDL

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Pose, Point, Quaternion

# import message_filters


class Publishsers():
    def make_pan(self, angle):
        pan_msg = Float64()
        pan_msg.data = angle
        self.dxl_pub.publish(pan_msg)


class Subscribe(Publishsers):
    def __init__(self):

        self.tf_listener = tf.TransformListener()

        # try:
        #     self.tf_listener.waitForTransform('/pan_link', '/target', rospy.Time(), rospy.Duration(1000.0))
        #     self.base_frame = 'base_footprint'
        # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        #     rospy.loginfo("Cannot find transform between /pan_link and /target")
        #     rospy.signal_shutdown("tf Exception")

        # Declaration Publisher
        self.dxl_pub = rospy.Publisher("/ubiquitous_display/pan_controller/command", Float64, queue_size = 10)

        # Declaration Subscriber
        self.pan_sub = rospy.Subscriber('/navigation/pan', String, self.callback)

    ###  ###
    def callback(self, msg):
        (position, rotation) = self.get_odom()
        # print rotation[2]
        self.make_pan(-rotation)


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/pan_link', '/target', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))


    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res

if __name__ == '__main__':
    rospy.init_node('detect_person_server')

    Subscribe = Subscribe()

    # rospy.Subscriber('/people_tracker_measurements', PositionMeasurementArray , server.ptm_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, server.pose_callback)

    rospy.spin()
