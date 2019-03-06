#!/usr/bin/env python
import rospy
import rospkg
import numpy as np

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()

class robot_tf_listener:

    def __init__(self):

        self.px = .0
        self.py = .0
        self.theta = .0
        self.ds = .2 # hard code here, should come from odometry
        self.dtheta = .5 # hard code here, should come from odometry 

        self.goal_pose_init()

        rospy.init_node('robot_tf_listener', anonymous=True)

        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.tf_lis()
            self.rate.sleep()

    def goal_pose_init(self):

        self.goalPose=PoseStamped()# goal pose, in odom frame
        self.goalPoseLocal = PoseStamped()# goal pose, in local base_link frame
        
        self.goalPose.header.frame_id = "odom"
        self.goalPoseLocal.header.frame_id = "base_link"

        self.goalPose.pose.position.x = 2.0
        self.goalPose.pose.position.y = 2.0
        self.goalPose.pose.position.z = .0
        
        # quat = quaternion_from_euler(.0, .0, .0)
        # Hardcode in angle in quaternion (0,0,1,0)
        self.goalPose.pose.orientation.x = 0
        self.goalPose.pose.orientation.y = 0
        self.goalPose.pose.orientation.z = 1
        self.goalPose.pose.orientation.w = 0


    def tf_lis(self):
        self.listener.waitForTransform("odom","base_link",rospy.Time(0),rospy.Duration(2.0))
        (trans,rot) = self.listener.lookupTransform("odom", "base_link", rospy.Time(0))
        
        self.goalPoseLocal = self.listener.transformPose("base_link",self.goalPose)
        print([self.goalPoseLocal.pose.position.x,self.goalPoseLocal.pose.position.y])


if __name__ == '__main__':
    try:
        tf = robot_tf_listener()

    except (rospy.ROSInterruptException):
        pass
