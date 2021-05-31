import rclpy
import math
import tf2_ros
#from suii_msgs.srv import DistanceToGoal
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros import LookupTransform


class EuclideanDistanceCalculator(Node):
    def __init__(self):
        super().__init__('euclidean_calculator_node')
        rclpy.create_node('euclidean_calculator_node')
        self.create_service(DistanceToGoal ,'get_distance',self.get_distance_handler)
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer, self)
        self.locations = None

        

    def get_distance_handler(self, waypoint_data):
        distance = self.calculate_euclidean_distance(waypoint_data.x, waypoint_data.y)
        return distance         


    def calculate_euclidean_distance(self, x_goal, y_goal):
        (linear, angular) = self.tf_listener.lookupTransform('base_link', 'map', rclpy.time(0))
        current_x = linear[0]
        current_y = linear[1]

        distance = abs(math.sqrt( ( (x_goal - current_x) ** 2) + ( (y_goal - current_y) ** 2) ) )
        return distance


def main(args=None):
    rclpy.init(args=args)

    euclidean = EuclideanDistanceCalculator()

    rclpy.spin(euclidean)

    rclpy.shutdown()


if __name__ == '__main__':
    main()