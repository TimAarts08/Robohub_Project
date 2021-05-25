
import rclpy
import time
import serial
import math

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
#from std_srvs.srv import EmptyResponse
from tf2_ros import transform_broadcaster
#from tf2_ros import quaternion_from_euler
#from rclpy import timer


#create a subscriber for the Odrive 
class Motors(object):
    def _init_(self):
        super().init_node = ("Odrive_node")
        self.odrv0 = None
        self.odrv1 = None

        self.ready = False

        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/right_front", self.runDirectVel, 10, callback_args= 0)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/left_front", self.runDirectVel, 10, callback_args= 1)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/right_back", self.runDirectVel, 10, callback_args= 2)
        self.subscription = self.create_subscription(Int32, "/cmd_vel_direct/left_back", self.runDirectVel, 10, callback_args= 3)
        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.runVel, 10)
        #callback means to which class should you give te information when you have it 
        # rospy.Subscriber("/cmd_vel_direct/right_front", Int32, callback=self.runDirectVel, callback_args=0)
        # rospy.Subscriber(topic, string, callback

        # ros2 subscriber (string, topic, self.listener_callback, queue size?)
        self.stateEncoder = [0.0,0.0,0.0,0.0]
        self.stateZ = 0.0
        self.odom_msg = Odometry()
        print(self.odom_msg)
        self.odom_topic = self.create_publisher(Odometry, "/odom", 1)

        self.tf2_broadcast = transform_broadcaster(queue_size=10)

       #The rclpy.timer has to get checked
        rclpy.timer.Timer(callback=self.getOdomDelta, nsecs=20000, secs=0)

        self.srv = self.create_service(Empty, "/motors/bootup",self.calibrateMotors)
        self.srv = self.create_service(Empty, "/motors/reboot", self.rebootMotors)
        

        rclpy.spin()

if __name__ == "__main__":
    m = Motors()