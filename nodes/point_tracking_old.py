#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, Pose

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

## GLOBAL VARIABLES

# Current x, y, theta
current_x = 0.0
current_y = 0.0
current_theta = 0.0

# Goal x, y, theta
goal_x = 0.0
goal_y = 0.0
goal_theta = 0.0

# Initiate k-values according to stability rules
kp = 0.5
ka = 0.7
kb = -1.0

# Initiate values
rho = 0.0
alpha = 0.0
beta = 0.0

## END GLOBAL VARIABLES

def odom_callback(data):
    global current_x, current_y, current_theta
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose)
    
    # Get values from odom
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    current_theta = euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])

    # Make delta values to simplify calculations
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y

    # Calculate rho, alpha, and beta
    rho = np.math.sqrt(delta_x ** 2 + delta_y ** 2)
    rospy.loginfo("rho: %s", rho)
    alpha = -current_theta + np.math.atan2(-1 * delta_y, -1 * delta_x) 
    rospy.loginfo("alpha: %s", alpha)
    beta = -1 * alpha - current_theta
    rospy.loginfo("beta: %s", beta)

    # Calculate linear(v) and angular(w)
    linear = kp * rho
    angular = ka * alpha + kb * beta

    # Publishing v & w to /cmd_vel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pub.publish(cmd)
    rate.sleep()

def goal_callback(data):
    global goal_x, goal_y, goal_theta
    # Get values from goal_pose
    goal_x = data.position.x
    goal_y = data.position.y
    quat = data.orientation
    euler = euler_from_quaternion(quat)
    goal_theta = euler[2] # z-value of the orientation quat

def point_tracking():
    print("point tracking\n")
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/goal_pose', Pose, goal_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('point_tracking', anonymous = True)

    rate = rospy.Rate(200.0) # 200 Hz
    rospy.sleep(5)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        point_tracking()
    except rospy.ROSInterruptException:
        pass
        