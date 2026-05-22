#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap
import os

MAP_SAVE_PATH = os.path.expanduser(
    "~/ros2_ws/src/com2009_team09_2026/maps/arena_map"
)

SAVE_INTERVAL_SEC = 20.0  # save map every 20 seconds

class MapSaverNode(Node):

    def __init__(self):
        super().__init__("task3_map_saver")

        self.shutdown = False

        # Service client for map saver
        self.cli = self.create_client(SaveMap, "/map_saver/save_map")

        # Wait for the service to become available
        self.get_logger().info("Waiting for /map_saver/save_map service...")
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Still waiting for map_saver service...")

        self.get_logger().info("Map saver service ready")

        # Timer to save map periodically
        self.save_timer = self.create_timer(
            timer_period_sec=SAVE_INTERVAL_SEC,
            callback=self.save_map,
        )

        # Save one immediately after a short delay
        self.initial_timer = self.create_timer(
            timer_period_sec=5.0,
            callback=self.initial_save,
        )

    def initial_save(self):
        """Save once shortly after startup, then cancel this timer."""
        self.save_map()
        self.initial_timer.cancel()

    def save_map(self):
        """Send a request to the map saver service."""
        self.get_logger().info(f"Saving map to: {MAP_SAVE_PATH}")

        request = SaveMap.Request()
        request.map_topic = "map"
        request.map_url = MAP_SAVE_PATH
        request.image_format = "png"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        future = self.cli.call_async(request)
        future.add_done_callback(self.save_map_callback)

    def save_map_callback(self, future):
        try:
            response = future.result()
            if response.result:
                self.get_logger().info("Map saved successfully!")
            else:
                self.get_logger().warn("Map save returned False - map may be incomplete yet.")
        except Exception as e:
            self.get_logger().error(f"Map save failed: {e}")

    def on_shutdown(self):
        self.get_logger().info("Final map save before shutdown...")
        # Synchronous final save attempt
        request = SaveMap.Request()
        request.map_topic = "map"
        request.map_url = MAP_SAVE_PATH
        request.image_format = "png"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self.cli.call_async(request)
        self.shutdown = True


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received shutdown")
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()