#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import math
import tf
import numpy as np

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class botControl:

    def __init__(self):
        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE_MODE"
        #self.control_mode = "MANUAL_CONTROL_MODE"
        self.firstIter = True
        self.encoder_initial_L = 0
        self.encoder_initial_R = 0
        self.encoder_adjusted_L = 0
        self.encoder_adjusted_R = 0
        self.last_encoder_measurementL = 0
        self.last_encoder_measurementR = 0
        self.diffEncoderL = 0
        self.diffEncoderR = 0
        self.theta = 0

        # setup xbee communication, change ttyUSB0 to the USB port dongle is in
        if (self.robot_mode == "HARDWARE_MODE"):
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        print("Xbee setup successful")
        self.address = '\x00\x0C'#you may use this to communicate with multiple bots

        # init an odometry instance, and configure odometry info
        self.odom_init()

        # init an ir instance, and configure ir info
        self.ir_init()
        
        # init log file, "False" indicate no log will be made, log will be in e190_bot/data folder
        self.log_init(data_logging=False,file_name="log.txt")

        rospy.init_node('botControl', anonymous=True)

        # Subscribe botControl node to /cmd_vel
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        self.pubOdom = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Publish ir (distance) sensor stuff
        # self.pubDistL = rospy.Publisher('/distL', ir_sensor, queue_size=10)
        # self.pubDistC = rospy.Publisher('/distC', ir_sensor, queue_size=10)
        # self.pubDistR = rospy.Publisher('/distR', ir_sensor, queue_size=10)
        self.pubDists = rospy.Publisher('/dists', Vector3, queue_size=10)

        self.time = rospy.Time.now()
        self.count = 0

        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.odom_pub()
            self.rate.sleep()

    def ir_init(self):
        # self.ir_L = ir_sensor()
        # self.ir_C = ir_sensor()
        # self.ir_R = ir_sensor()
        self.ir_L = 0
        self.ir_C = 0
        self.ir_R = 0

    def odom_init(self):
        self.Odom = Odometry()
        self.Odom.header.frame_id = "/odom"
        self.Odom.child_frame_id = "/base_link"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.wheel_radius = 0.0345 # in meters
        self.bot_radius = 0.0705 # in meters

    def log_init(self,data_logging=False,file_name="log.txt"):
        self.data_logging=data_logging
        if(data_logging):
            self.file_name = file_name
            self.make_headers()

    def cmd_vel_callback(self,CmdVel):
        if(self.robot_mode == "HARDWARE_MODE"):

            right_alpha = 0.775 / 255
            left_alpha = 0.8 / 255

            RPWM =int((CmdVel.linear.x - CmdVel.angular.z * self.bot_radius)/right_alpha)
            LPWM =int((CmdVel.linear.x + CmdVel.angular.z * self.bot_radius)/left_alpha)
            print(RPWM)
            print(LPWM)

            RDIR = int(RPWM > 0)
            LDIR = int(LPWM > 0)

            RPWM = abs(RPWM)
            LPWM = abs(LPWM) 

            # Combine command and send to terminal and robot
            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            # print(command)
            self.xbee.tx(dest_addr = self.address, data = command)

    def odom_pub(self):
        if(self.robot_mode == "HARDWARE_MODE"):
            self.count = self.count + 1
            print(self.count) # second counter
            command = '$S @'
            self.xbee.tx(dest_addr = self.address, data = command)
            try:
                update = self.xbee.wait_read_frame()
            except:
                pass

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:] #encoder readings are here, 2d array

            # Get initial encoder values for subtracting from ongoing measurements
            if(self.firstIter == True):
                self.encoder_initial_L = encoder_measurements[1] # check
                self.encoder_initial_R = encoder_measurements[0]
                # Prevent value 'jumping'
                self.diffEncoderL = 0
                self.diffEncoderR = 0
            
            # Adjusted encoder values
            self.encoder_adjusted_L = encoder_measurements[1] - self.encoder_initial_L
            self.encoder_adjusted_R = encoder_measurements[0] - self.encoder_initial_R

            #print ("update sensors measurements ",encoder_measurements, range_measurements)

            #how about velocity?
            time_diff = rospy.Time.now() - self.time
            self.diffEncoderL = self.encoder_adjusted_L - self.last_encoder_measurementL
            self.diffEncoderR = self.encoder_adjusted_R - self.last_encoder_measurementR
            self.last_encoder_measurementL =  self.encoder_adjusted_L
            self.last_encoder_measurementR =  self.encoder_adjusted_R

            self.firstIter = False

            sL = -float(2*np.math.pi*self.wheel_radius*self.diffEncoderL)/1440
            sR = -float(2*np.math.pi*self.wheel_radius*self.diffEncoderR)/1440
            # print(self.diffEncoderL)
            # print(self.diffEncoderR)
            deltaS = (sR + sL)/2
            # print("Before" + str(deltaS)) 
            deltaTheta = (sR - sL)/(2 *self.bot_radius)

            self.Odom.pose.pose.position.x = deltaS*math.cos(self.theta + deltaTheta/2)+self.Odom.pose.pose.position.x
            self.Odom.pose.pose.position.y = deltaS*math.sin(self.theta + deltaTheta/2)+self.Odom.pose.pose.position.y
            self.Odom.pose.pose.position.z = .0
            self.theta = self.theta + deltaTheta
            quat = quaternion_from_euler(.0, .0, self.theta)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]
            

            # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            self.odom_broadcaster.sendTransform(
                (self.Odom.pose.pose.position.x, self.Odom.pose.pose.position.y, .0),
                tf.transformations.quaternion_from_euler(.0, .0, self.theta),
                rospy.Time.now(),
                self.Odom.child_frame_id,
                self.Odom.header.frame_id,
            )

            self.pubOdom.publish(self.Odom) #we publish in /odom topic

            #about range sensors, update here
            range_measurements = data[:-2] #range readings are here, 3d array
            self.pubRangeSensor(range_measurements)

        if(self.data_logging):
            self.log_data()

        self.time = rospy.Time.now()

    def ir_cal(self, ADC):
        # Calibrate IR sensor measurements
        # print(ADC)

        # If ADC = 0, then distance calibration crashes so avoid that
        if(ADC == 0):
            return 100
        else:
            distance = -1 * math.log(ADC/1022.0) / 2.18
            # print(distance)
            return distance

    def pubRangeSensor(self,ranges):
        
        #Maybe you want to calibrate them now? Make a new function called "ir_cal"
        self.ir_L = self.ir_cal(ranges[0])
        self.ir_C = self.ir_cal(ranges[1])
        self.ir_R = self.ir_cal(ranges[2])

        
        # self.pubDistL.publish(self.ir_L)
        # self.pubDistC.publish(self.ir_C)
        # self.pubDistR.publish(self.ir_R)
        self.pubDists.publish(self.ir_L, self.ir_C, self.ir_R)


    def make_headers(self):
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')
        f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.close()

    def log_data(self):
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        # edit this line to have data logging of the data you care about
        data = [str(x) for x in [1,2,3,self.Odom.pose.pose.position.x,self.Odom.pose.pose.position.y]]
        
        f.write(' '.join(data) + '\n')#maybe you don't want to log raw data??
        f.close()

if __name__ == '__main__':
    try:
        bot = botControl()

    except rospy.ROSInterruptException:
        pass