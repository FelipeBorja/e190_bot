#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Initiate k-values according to stability rules
kp = 3
ka = -3
kb = 5

# Initiate values
rho = 0.0
alpha = 0.0
beta = 0.0

# Goal x and y, hardcoded for now
init_goal_x = 1.0
init_goal_y = 1.0

def odom_callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose)
    
    # Get values from odom
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    theta = euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])

    # Make delta values to simplify calculations
    delta_x = goal_x - current_x
    delta_y = goal_y - current_y

    # Calculate rho, alpha, and beta
    rho = math.sqrt(delta_x**2 + delta_y**2)
    alpha = -1.0*theta + math.atan2(-1.0*delta_y, -1.0*delta_x) # ERROR HERE
    beta = -theta - alpha

    # Calculate v and w
    v = kp * rho
    w = ka * alpha + kb * beta

    # Publishing a standardized message! 
    standardMessage = Twist()
    standardMessage.linear = Vector3(v, 0, 0)
    standardMessage.angular = Vector3(0, 0, w)
    
    # This will publish the velocity & angular information each time! 
    pub.publish(standardMessage)

def goal_callback(data):
    # Get values from goal_pose
    goal_x = data.pose.pose.position.x
    goal_y = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    theta = euler_from_quaternion([quat.x,quat.y,quat.z, quat.w])

def point_tracking():
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/goal_pose', Pose, goal_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('point_tracking', anonymous = True)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        point_tracking()
    except rospy.ROSInterruptException:
        pass
        