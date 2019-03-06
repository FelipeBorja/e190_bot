#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry, Path

# Initiate k-values according to stability rules
kp = 3.0
ka = -3.0
kb = 5.0

goal_x = 0.0
goal_y = 0.0

def goal_callback(path):
    goal_x = path.poses.pose.position.x
    goal_y = path.poses.pose.position.y

def point_tracking():
    print("point_tracking begins")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/goal', Path, goal_callback)
    rospy.init_node('point_tracking', anonymous = True)
    listener = tf.TransformListener()

    rate = rospy.Rate(200.0) # 200 Hz
    rospy.sleep(5)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # Modified from simple.tf
            listener.waitForTransform("/base_link", "/goal", now, rospy.Duration(2.0))
            (trans,rot) = listener.lookupTransform("/base_link", "/goal", now)

            # Get the theta value this way from tf functions. 
            theta = tf.transformations.euler_from_quaternion(rot)[2]
            delta_x = goal_x - trans[0]
            delta_y = goal_y - trans[1]

            # Get the rho, beta, and alpha values from equations
            rho = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            beta = -1 * math.atan2(-1 * delta_y, -1 * delta_x)
            alpha = -1 * (beta - theta)

            # Calculate linear(v) and angular(w)
            linear = kp * rho
            angular = ka * alpha + kb * beta

            # Publishing v & w to /cmd_vel
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            pub.publish(cmd)
            rate.sleep()
    except(tf.LookupException,tf.ConnectivityException):
        pass

if _name_ == '_main_':
    try:
        point_tracking()
    except rospy.ROSInterruptException:
        pass
