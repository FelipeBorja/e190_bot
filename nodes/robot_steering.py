#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Twist, Vector3, Transform, Quaternion
from fiducial_msgs.msg import FiducialTransformArray


class robot_steering():

    def __init__(self):
        # Initialize node, publisher, and subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rospy.init_node('robot_steering', anonymous = True)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.robot_steering_callback)
        
        rate = rospy.Rate(10) # 10 Hz

        self.vel = Twist() 
        self.vel.linear = Vector3(0,0,0)
        self.vel.angular = Vector3(0,0,0)

        while not rospy.is_shutdown():
            self.pub.publish(self.vel)
            rate.sleep()

        
    def robot_steering_callback(self, fid_trans):
        if len(fid_trans.transforms) > 0:
            fid_linear = fid_trans.transforms[0].transform.translation.z
            fid_rot = fid_trans.transforms[0].transform.rotation
            rot = np.array([fid_rot.x, fid_rot.y, fid_rot.z, fid_rot.w])

            rot_Euler = tf.transformations.euler_from_quaternion(rot)[2]

            self.vel.linear.x = 0.5 * fid_linear 
            self.vel.angular.z = 1.0 * rot_Euler 
            

if __name__ == '__main__':
    try:
        robot_steering()
    except rospy.ROSInterruptException:
        pass
