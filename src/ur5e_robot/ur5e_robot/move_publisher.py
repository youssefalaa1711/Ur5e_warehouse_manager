#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim.msg import Pose


class CoordinatesPublisher(Node):
    def __init__(self):
        super().__init__("Coordinates_Publisher_Node")
        self.publisher_ = self.create_publisher(Point, 'slot_coordinates', 10)
        self.timer = self.create_timer(1.0, self.publish_coordinates)
        ##self.initial_pose_publisher_ = self.create_publisher(Pose, '/turtle1/initialpose', 10)


    def publish_coordinates(self):
        slot_1 = Point()
        slot_1.x = 1.0
        slot_1.y = 5.0

        slot_2 = Point()
        slot_2.x = 4.0
        slot_2.y = 8.0

        slot_3 = Point()
        slot_3.x = 7.0
        slot_3.y = 2.0

        slot_4 = Point()
        slot_4.x = 9.0
        slot_4.y = 1.0

        self.publisher_.publish(slot_1)
        self.publisher_.publish(slot_2)
        self.publisher_.publish(slot_3)
        self.publisher_.publish(slot_4)


        initial_pose = Pose()
        initial_pose.x = 0.0
        initial_pose.y = 0.0
        initial_pose.theta = 0.0
        ##self.initial_pose_publisher_.publish(initial_pose)

def main(args=None):
    rclpy.init(args=args)
    coordinates_publisher_node = CoordinatesPublisher()
    rclpy.spin(coordinates_publisher_node)
    coordinates_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
