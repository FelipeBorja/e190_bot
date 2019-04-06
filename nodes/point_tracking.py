#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf

from e190_bot.srv import *
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from nav_msgs.msg import Odometry, Path

goal_x = .0 
goal_y = .0 
goal_reached = False

def goal_callback(point):
    global goal_x, goal_y
    goal_x = point.pose.position.x
    goal_y = point.pose.position.y
    #print("goal x" + str(goal_x))

def handle_point_tracking(req):
    global goal_x, goal_y, goal_reached
    goal_x = req.goal.position.x
    goal_y = req.goal.position.y
    return PointTrackingResponse(goal_reached)
    
def point_tracking():
    global goal_x, goal_y, goal_reached
    print("point_tracking begins")

    # Get k-values from launch file
    kp = rospy.get_param("kp", default = 1.0)
    ka = rospy.get_param("ka", default = 2.0)
    kb = rospy.get_param("kb", default = -0.7)
    print "k-values: [kp = %s ka = %s kb = %s]"%(kp, ka, kb)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.Subscriber('/goal', PoseStamped, goal_callback)  # not needed since we are using rosservice now
    rospy.init_node('point_tracking', anonymous = True)
    rospy.Service('point_tracking_service', PointTracking, handle_point_tracking)
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0) # 10 Hz
    # rospy.sleep(5)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            # Modified from simple.tf
            listener.waitForTransform("/base_link", "/odom", rospy.Time(), rospy.Duration(2.0))
            (trans,rot) = listener.lookupTransform("/odom", "/base_link", rospy.Time())

            # Get the theta value this way from tf functions. 
            theta = tf.transformations.euler_from_quaternion(rot)[2]
            delta_x = goal_x - trans[0]
            delta_y = goal_y - trans[1]
            #rospy.loginfo("delta_x: %s", delta_x)
            #rospy.loginfo("delta_y: %s", delta_y)

            # Get the rho, beta, and alpha values from equations
            rho = math.sqrt(delta_x ** 2 + delta_y ** 2)
            # rospy.loginfo("rho: %s", rho)
            beta = -1 * math.atan2(delta_y, delta_x)
            # rospy.loginfo("beta: %s", beta)
            alpha = -1 * (beta - theta)
            print(theta)
            # rospy.loginfo("alpha: %s", alpha)

            # Calculate linear(v) and angular(w)
            linear = kp * rho
            angular = ka * alpha + kb * beta

            # Stopping the robot
            if(rho < 0.2):
                print("STOP")
                goal_reached = True
                linear = 0.0
                angular = 0.0
            else:
                goal_reached = False

            # Publishing v & w to /cmd_vel
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            pub.publish(cmd)
            rate.sleep()
        except(tf.LookupException,tf.ConnectivityException):
            print("throw")
            pass

if __name__ == '__main__':
    try:
        point_tracking()
    except rospy.ROSInterruptException:
        pass
