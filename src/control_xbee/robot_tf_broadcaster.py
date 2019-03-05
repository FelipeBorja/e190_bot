#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import math

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class robot_tf_broadcaster:

    def __init__(self):

        self.px = .0
        self.py = .0
        self.theta = .0
        self.ds = .2 # hard code here, should come from odometry
        self.dtheta = .5 # hard code here, should come from odometry 

        rospy.init_node('robot_tf_broadcaster', anonymous=True)
        self.odom_broadcaster = tf.TransformBroadcaster()
        # Broadcasters for each of the IR sensors
        self.cam_static_broadcaster1 = tf.TransformBroadcaster()
        self.cam_static_broadcaster2 = tf.TransformBroadcaster()
        self.cam_static_broadcaster3 = tf.TransformBroadcaster()

        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.tf_pub()
            self.rate.sleep()


    def tf_pub(self):
        # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        self.odom_broadcaster.sendTransform(
            (self.px, self.py, .0),
            tf.transformations.quaternion_from_euler(.0, .0, self.theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        self.cam_static_broadcaster1.sendTransform(
            (-0.1, 0.0, 0.0),
            tf.transformations.quaternion_from_euler(.0, (math.radians(90)), 0.0),
            rospy.Time.now(),
            "camera1",
            "base_link"
        )

        self.cam_static_broadcaster2.sendTransform(
            (0.0, 0.1, 0.0),
            tf.transformations.quaternion_from_euler(.0, (math.radians(90)), (math.radians(90))),
            rospy.Time.now(),
            "camera2",
            "base_link"
        )

        self.cam_static_broadcaster3.sendTransform(
            (0.1, -0.1, 0.0),
            tf.transformations.quaternion_from_euler(.0, (math.radians(90)), (math.radians(-90))),
            rospy.Time.now(),
            "camera3",
            "base_link"
        )

        self.px = self.ds * np.cos(self.theta+self.dtheta/2) + self.px
        self.py = self.ds * np.sin(self.theta+self.dtheta/2) + self.py
        self.theta += self.dtheta

if __name__ == '__main__':
    try:
        tf = robot_tf_broadcaster()

    except rospy.ROSInterruptException:
        pass