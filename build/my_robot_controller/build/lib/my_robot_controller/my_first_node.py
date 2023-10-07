#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.get_logger().info("Hello from ROS 2")


def main(args=None):
    rclpy.init(args=args) #start ros2 communications

    node = MyNode()
    rclpy.spin(node=node) #keeps running unitl killing it manually



    rclpy.shutdown() #shutdone ros2 communications

if __name__ == "__main__":
    main()