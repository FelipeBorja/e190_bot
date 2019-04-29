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

        self.inactive_timer = 0

        self.vel = Twist() 
        self.vel.linear = Vector3(0,0,0)
        self.vel.angular = Vector3(0,0,0)

        while not rospy.is_shutdown():
            self.pub.publish(self.vel)
            rate.sleep()

        
    def robot_steering_callback(self, fid_trans):
        if len(fid_trans.transforms) > 0:
            self.inactive_timer = 0

            fid_linear = fid_trans.transforms[0].transform.translation.z
            fid_rot = fid_trans.transforms[0].transform.rotation
            rot = np.array([fid_rot.x, fid_rot.y, fid_rot.z, fid_rot.w])

            rot_Euler = tf.transformations.euler_from_quaternion(rot)[2]

            # This is a simple linear velocity command
            self.vel.linear.x = 0.5 * fid_linear + 0.2

            # This linear velocity command makes:
            # closer tag -> faster speed, farther tag -> slower speed
            #self.vel.linear.x = 2.10 - 2.5 * fid_linear
            print(fid_linear)
            self.vel.angular.z = 1.2 * rot_Euler
        else:
            self.inactive_timer += 1
            if self.inactive_timer > 5:
                # If no fiducial transforms are detected after 1 sec, stop.
                self.vel.linear.x = 0
                self.vel.angular.z = 0
            

if __name__ == '__main__':
    try:
        robot_steering()
    except rospy.ROSInterruptException:
        pass
