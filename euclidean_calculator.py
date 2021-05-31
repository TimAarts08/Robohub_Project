import rclpy
import math
import tf2_ros
from suii_msgs.srv import DistanceToGoal    
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros import LookupTransform


class EuclideanDistanceCalculator(Node):
    def __init__(self):
        super().__init__('euclidean_calculator_node')                                       #creates a node called euclidean_calculator_node
        self.create_service(DistanceToGoal ,'get_distance',self.get_distance_handler)       #creates a service that calls for the get_distance_handler function
        self.buffer = tf2_ros.Buffer()                                                      #creates a buffer variable for the tf2 listener
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self)                     #creates a tf2 listener for the coordinates of the robot
        self.locations = None

        

    def get_distance_handler(self, waypoint_data):
        distance = self.calculate_euclidean_distance(waypoint_data.x, waypoint_data.y)      #makes a variable that calls for the function calculate_euclidean_distance
        return distance         


    def calculate_euclidean_distance(self, x_goal, y_goal):
        (linear, angular) = self.tf_listener.lookupTransform('base_link', 'map', rclpy.time(0)) #receives the coordinates of the robot in the map
        current_x = linear[0]
        current_y = linear[1]

        distance = abs(math.sqrt( ( (x_goal - current_x) ** 2) + ( (y_goal - current_y) ** 2) ) )      #calculates the distance from the robot to the goal
        return distance


def main(args=None):
    rclpy.init(args=args)

    EuclideanDistanceCalculator()

    rclpy.shutdown()


if __name__ == '__main__':
    main()