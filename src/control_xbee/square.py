#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3

# change these specs to get velocities
forward_velocity = 0.5 # m/sec
rotate_velocity = math.pi # rad/sec

def square():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('square', anonymous = True)

    pause = Twist()
    pause.linear = Vector3(0, 0, 0)
    pause.angular = Vector3(0, 0, 0)

    forward = Twist()
    forward.linear = Vector3(forward_velocity, 0, 0)
    forward.angular = Vector3(0, 0, 0)

    rotate = Twist()
    rotate.linear = Vector3(0, 0, 0)
    rotate.angular = Vector3(0, 0, rotate_velocity)

    cmds = [pause, forward, pause, rotate]*4 + [pause]
    cmd_step = 0
    sleep_times = {pause: 0.5, forward: 2, rotate: 0.5}

    while not rospy.is_shutdown() and cmd_step < len(cmds):
        cmd = cmds[cmd_step]
        sleep_time = sleep_times[cmd]
        # rospy.loginfo(cmd, sleep_time)
        pub.publish(cmd)
        rospy.sleep(sleep_time)
        cmd_step += 1

if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException:
        pass