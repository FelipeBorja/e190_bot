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
        rospy.init_node('robot_steering', anonymous = True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.robot_steering_callback)
    
    
    def robot_steering_callback(self, fid_trans):
        if(fid_trans.transforms):
            trans = fid_trans.transforms.transform.translation
            rot = fid_trans.transforms.transform.rotation
            

        

    




if __name__ == '__main__':
    try:
        robot_steering()
    except rospy.ROSInterruptException:
        pass
