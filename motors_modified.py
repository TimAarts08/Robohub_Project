import rclpy
import time
import serial
# import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from tf2_ros import transform_broadcaster
# from tf2_ros import quaternion_from_euler
#from rclpy import timer

class Motors(object):
    def _init_(self):
        super().init_node = ("Odrive_node")
        self.odrv0 = None
        self.odrv1 = None

        self.ready = False

        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/right_front", self.runDirectVel, callback_args= 0)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/left_front", self.runDirectVel, callback_args= 1)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/right_back", self.runDirectVel, callback_args= 2)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/left_back", self.runDirectVel, callback_args= 3)
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.runVel)

        self.stateEncoder = [0.0,0.0,0.0,0.0]
        self.stateZ = 0.0
        self.odom_msg = Odometry()

        self.odom_topic = self.create_publisher(Odometry, "/odom", 1)

        self.tf2_broadcast = transform_broadcaster(10)
        # self.tf2_broadcast = transform_broadcaster(queue size=10)

        # rospy.Timer(rospy.Duration(secs=0, nsecs=20000), callback=self.getOdomDelta)
        # #  rospy.Timer(rospy.Duration(2), my_callback)

        self.timer = self.create_timer(nsecs=20000, callback = self.getOdomDelta)


        self.srv = self.create_service(Empty, "/motors/bootup",self.calibrateMotors)
        self.srv = self.create_service(Empty, "/motors/reboot", self.rebootMotors)

        rclpy.spin
        

    def bootup(self):
        self.get_logger().info("ODrives loaded")   #logging to the info "barocity?"
        time.sleep(1)
        self.odrv0.write("w axis0.requested_state 3\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.requested_state 3\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.requested_state 3\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.requested_state 3\n")
        time.sleep(2)
        self.checkState()

        time.sleep(0.01)
        self.odrv0.write("w axis0.requested_state 8\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.requested_state 8\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.requested_state 8\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.requested_state 8\n")
        time.sleep(0.01)

        self.odrv0.write("w axis0.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv0.write("w axis1.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv1.write("w axis0.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.odrv1.write("w axis1.controller.config.control_mode 2\n")
        time.sleep(0.01)
        self.get_logger().info("ODrives bootup done")

    
    def checkState(self):

        # Create array tryFlags 
        # while statement --> same as while True (see array values)
        # If statment --> if True (see array values)
        # Write current state of odrive axis 0 to variable self.odrv0
        # Make variable dat equal to self.odrv0
        # Checks the state of the Odrive (Idling or not)
        # wait time
        # next Odrv motor ..

        tryFlags = [True, True, True, True]
        while(tryFlags[0] or tryFlags[1] or tryFlags[2] or tryFlags[3]): 
            if(tryFlags[0]):
                self.odrv0.write("r axis0.current_state\n") 
                dat = self.odrv0.readline()
                if(dat == "1\r\n"):
                    tryFlags[0] = False
                time.sleep(0.1)
            if(tryFlags[1]):
                self.odrv0.write("r axis1.current_state\n")
                dat = self.odrv0.readline()
                if(dat == "1\r\n"):
                    tryFlags[1] = False
                time.sleep(0.1)
            if(tryFlags[2]):
                self.odrv1.write("r axis0.current_state\n")
                dat = self.odrv1.readline()
                if(dat == "1\r\n"):
                    tryFlags[2] = False
                time.sleep(0.1)
            if(tryFlags[3]):
                self.odrv1.write("r axis1.current_state\n")
                dat = self.odrv1.readline()
                if(dat == "1\r\n"):
                    tryFlags[3] = False
                time.sleep(0.1)

    def calibrateMotors(self, dat):
        
        # Variable Odrv connecting with Serial port with a certain baudrate
        # Wait time
        # Giving variable self.Odrv a value which is recognized by the Odrive as rebooting.
        # wait time
        # re-establish the connection
        # Wait time
        # Execute bootup function
        # Set self.ready variable HIGH

        self.odrv0 = serial.Serial("/dev/ttyODRV_F", 115200)
        self.odrv0 = serial.Serial("/dev/ttyODRV_B", 115200)
        time.sleep(2)
        self.odrv0.write("sb\n")
        self.odrv1.write("sb\n")
        time.sleep(5)
        self.odrv0 = serial.Serial("/dev/ttyODRV_F", 115200)
        self.odrv1 = serial.Serial("/dev/ttyODRV_B", 115200)
        time.sleep(2)
        self.bootup()
        self.ready = True
        return EmptyResponse()

    
    def rebootMotors(self, dat):
        if(self.ready):
            self.ready = False
            time.sleep(1)
            self.odrv0.write("sb\n")
            self.odrv1.write("sb\n")
            self.odrv0 = None
            self.odrv1 = None
            time.sleep(10)
            self.odrv0 = serial.Serial("/dev/ttyODRV_F",115200)
            self.odrv1 = serial.Serial("/dev/ttyODRV_B",115200)
            self.bootup()
            self.ready = True

            #de return staat in de vorige code 1x een backspace terug
            #oftewel in de def.
        return EmptyResponse()

