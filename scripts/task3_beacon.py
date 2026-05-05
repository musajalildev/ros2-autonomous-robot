#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class BeaconSearch(Node):
    def __init__(self):
        super().__init__("beacon_search")

        self.declare_parameter('target_colour', 'blue')
        colour = self.get_parameter('target_colour').get_parameter_value().string_value

        # Required log message - must appear within 10 seconds
        self.get_logger().info(f"TARGET BEACON: Searching for {colour}.")

def main(args=None):
    rclpy.init(args=args)
    node = BeaconSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()