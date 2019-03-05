#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class botControl:

    def __init__(self):
        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE_MODE"#"SIMULATION_MODE"
        #self.control_mode = "MANUAL_CONTROL_MODE"

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

        #init an odometry instance, and configure odometry info
        self.odom_init()
        
        #init log file, "False" indicate no log will be made, log will be in e190_bot/data folder
        self.log_init(data_logging=False,file_name="log.txt")

        rospy.init_node('botControl', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        self.pubOdom = rospy.Publisher('/odom', Odometry, queue_size=10)

        #self.pubDistL = rospy.Publisher('/distL', ir_sensor, queue_size=10)
        #self.pubDistC = rospy.Publisher('/distC', ir_sensor, queue_size=10)
        #self.pubDistR = rospy.Publisher('/distR', ir_sensor, queue_size=10)
        self.time = rospy.Time.now()
        self.count = 0

        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.odom_pub()
            self.rate.sleep()

    # def ir_init(self):
    #     self.ir_L = ir_sensor()
    #     self.ir_C = ir_sensor()
    #     self.ir_R = ir_sensor()

    def odom_init(self):
        self.Odom = Odometry()
        self.Odom.header.frame_id = "odom_wheel"
        self.Odom.child_frame_id = "base_link"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.wheel_radius = .1 #unit in m, need re-measurement

    def log_init(self,data_logging=False,file_name="log.txt"):
        self.data_logging=data_logging
        if(data_logging):
            self.file_name = file_name
            self.make_headers()

    def cmd_vel_callback(self,CmdVel):
        if(self.robot_mode == "HARDWARE_MODE"):

            #update this!!
            if(CmdVel.angular.z < 0):
                LDIR = 0
                RDIR = 0
            else:
                LDIR = 1
                RDIR = 1

            RPWM =int(CmdVel.linear.x/1*255)
            LPWM =int(CmdVel.linear.x/1*255)

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            #print("subscriber working")

            #command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.xbee.tx(dest_addr = self.address, data = command)

    def odom_pub(self):
        if(self.robot_mode == "HARDWARE_MODE"):
            self.count = self.count + 1
            print(self.count)
            command = '$S @'
            self.xbee.tx(dest_addr = self.address, data = command)
            try:
                update = self.xbee.wait_read_frame()
            except:
                pass

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:] #encoder readings are here, 2d array

            #print ("update sensors measurements ",encoder_measurements, range_measurements)

            #Update here, the code is totally non-sense

            #how about velocity?
            time_diff = rospy.Time.now() - self.time
            #self.last_encoder_measurementL =  
            #self.last_encoder_measurementR =  
            #self.diffEncoderL = 
            #self.diffEncoderR = 
            
            self.Odom.pose.pose.position.x = encoder_measurements[0]/10000.0 #this won't work for sure
            self.Odom.pose.pose.position.y = encoder_measurements[1]/10000.0
            self.Odom.pose.pose.position.z = .0
            quat = quaternion_from_euler(.0, .0, .0)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]

            # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            self.odom_broadcaster.sendTransform(
                (self.Odom.pose.pose.position.x, self.Odom.pose.pose.position.y, .0),
                tf.transformations.quaternion_from_euler(.0, .0, 1.57),
                rospy.Time.now(),
                self.Odom.child_frame_id,
                self.Odom.header.frame_id,
            )

            self.pubOdom.publish(self.Odom) #we publish in /odom topic

            #about range sensors, update here
            range_measurements = data[:-2] #range readings are here, 3d array
            #self.pubRangeSensor(range_measurements)

        if(self.data_logging):
            self.log_data()

        self.time = rospy.Time.now()

    # def pubRangeSensor(self,ranges):
        
    #     #May be you want to calibrate them now? Make a new function called "ir_cal"
    #     self.ir_L.distance = ir_cal(ranges[0])
    #     self.ir_C.distance = ir_cal(ranges[1])
    #     self.ir_R.distance = ir_cal(ranges[2])
        
    #     self.pubDistL.publish(self.ir_L)
    #     self.pubDistC.publish(self.ir_C)
    #     self.pubDistR.publish(self.ir_R)

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