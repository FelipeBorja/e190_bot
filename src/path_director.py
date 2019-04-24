#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from e190_bot.srv import *

class path_director:
    
    def __init__(self):
        rospy.init_node('path_director',anonymous=True)
        rospy.Subscriber('/plan',Path, self.plan_callback)
        rospy.spin()

    def plan_callback(self, path):
        print("/plan callback has been reached.")
        #### calling service
        i = 0
        for pose_stamped in path.poses:
            pose = pose_stamped.pose
            print('tracking to node '+str(i))
            i+=1
            point_tracking_requester = rospy.ServiceProxy('Point_Tracking',PointTracking)
            hasReached = point_tracking_requester(pose)
            if not hasReached:
                print("Oh no! Something went wrong")
                break
            
if __name__=="__main__":
    try:
        path_director()
    except rospy.ROSInterruptException:
        pass
