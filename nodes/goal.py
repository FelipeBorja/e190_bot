#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Point
from nav_msgs.msg import Odometry, Path

def goal():
    pub = rospy.Publisher('/goal', PoseStamped, queue_size = 10)
    rospy.init_node('goal', anonymous = True)

    point1 = PoseStamped()
    point1.pose.position = Point(0.500, 0.000, 0.000)
    
    point2 = PoseStamped()
    point2.pose.position = Point(0.700, -0.400, 0.000)

    messagePath = Path()
    messagePath.poses.append(point1)
    messagePath.poses.append(point2)

    sleep_time = 1 # 1 Hz

    while not rospy.is_shutdown():
        pub.publish(point2)
        rospy.sleep(sleep_time)

if __name__ == '__main__':
    try:
        goal()
    except rospy.ROSInterruptException:
        pass