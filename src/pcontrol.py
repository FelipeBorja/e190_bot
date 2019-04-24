#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Vector3
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from math import cos, sin, sqrt, atan2, pi

from e190_bot.srv import *       

class pcontrol():
	def __init__(self):
		## state init  ##
		# gets overwritten upon recieving messages
		# pose global robot
		self.pgr = Odometry().pose.pose

                self.get_gains()
                
		rospy.init_node('proportional_control', anonymous=False)
		rospy.Subscriber('/odom', Odometry, self.pgr_callback)
		#rospy.Subscriber('/goal_pose', Point, self.pgg_callback)
		self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
		s = rospy.Service('Point_Tracking', PointTracking, self.handle_point_track)
		rospy.spin()

	def get_gains(self):
                self.kp_og = float(rospy.get_param('kp',.375)) #.375
                self.ka_og = float(rospy.get_param('ka',.5)) #.5
                self.kB_og = float(rospy.get_param('kb',-.25)) #-.25
                print('Gains: kp: '+str(self.kp_og)+', ka: '+str(self.ka_og)+', kb: '+str(self.kB_og))

	def handle_point_track(self, req):
                print("Tracking point...")
                print(req.pose)
                self.pgg = req.pose.position
                self.rate = rospy.Rate(2)
                done = False
		while not rospy.is_shutdown() and not done:
			done = self.cmd_vel_pub()
			self.rate.sleep()
		return True
		

	def pgr_callback(self, odom):
		self.pgr = odom.pose.pose

	#def pgg_callback(self, goal_pose):
	#	self.pgg = goal_pose

	def cmd_vel_pub(self):
		# We'll use the notation in lec8 slides
		theta = tf.transformations.euler_from_quaternion([
			self.pgr.orientation.x,
			self.pgr.orientation.y,
			self.pgr.orientation.z,
			self.pgr.orientation.w])[2]
		dx = self.pgg.x - self.pgr.position.x
		dy = self.pgg.y - self.pgr.position.y
		p = sqrt(dx**2 + dy**2)
		a = -theta + atan2(dy, dx)
		B = -theta - a

		# Gain scheduling to increase convergence as p->0
		if p>=.2:
			kp = self.kp_og
			ka = self.ka_og
			kB = self.kB_og
		else:
			kp = self.kp_og*1.75
			ka = self.ka_og*1.75
			kB = self.kB_og

		if a >= -pi/2 and a <= pi/2:
			v = kp*p
		else:
			a = -theta + atan2(-dy, -dx)
			B = -(a + theta)
			v = -kp*p
		w = ka*a + kB*B

		#print(self.pgr.position.x,self.pgr.position.y)
		if p >= .1:
			self.pub_cmd_vel.publish(
				Twist(
					Vector3(v,0,0),
					Vector3(0,0,w)))
			return False
		else:
			self.pub_cmd_vel.publish(
				Twist(
					Vector3(0,0,0),
					Vector3(0,0,0)))
                        return True

if __name__ == '__main__':
	try:
		pcontrol()
	except rospy.ROSInterruptException:
		pass
