#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from com2009_team09_2026.msg import ObstacleInfo
import numpy as np

OBSTACLE_THRESHOLD = 0.35
WALL_THRESHOLD = 0.75     
MIN_VALID_RANGE = 0.12                           

FRONT_ARC   = list(range(0, 26)) + list(range(335, 360))  
LEFT_ARC    = list(range(26, 91))                          
RIGHT_ARC   = list(range(270, 335))                        
REAR_ARC    = list(range(91, 270))                         

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__("obstacle_detector")

        self.get_logger().info("Obstacle detector node initialised.")

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.obstacle_pub = self.create_publisher(
            ObstacleInfo,
            "/obstacle_info",
            10
        )

        self.ranges = []

    def filter_ranges(self, ranges, indices):
        vals = []
        n = len(ranges)

        for i in indices:
            if i >= n:
                continue

            r = ranges[i]
            if r > MIN_VALID_RANGE and not np.isinf(r) and not np.isnan(r):
                vals.append(r)

        return vals if vals else [float("inf")]

    def classify_obstacle(self, ranges, front_indices):
        vals = self.filter_ranges(ranges, front_indices)

        if min(vals) == float("inf"):
            return "unknown"

        spread = max(vals) - min(vals)
        min_dist = min(vals)

        if spread > 0.20:
            return "cylinder"

        if spread <= 0.20:
            if min_dist > 1.0:
                return "arena_boundary"
            else:
                return "wall"

        return "unknown"

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges

        front_vals = self.filter_ranges(ranges, FRONT_ARC)
        left_vals  = self.filter_ranges(ranges, LEFT_ARC)
        right_vals = self.filter_ranges(ranges, RIGHT_ARC)

        front_dist = float(np.min(front_vals))
        left_dist  = float(np.min(left_vals))
        right_dist = float(np.min(right_vals))

        object_type = self.classify_obstacle(ranges, FRONT_ARC)

        msg_out = ObstacleInfo()
        msg_out.front_distance   = front_dist
        msg_out.left_distance    = left_dist
        msg_out.right_distance   = right_dist
        msg_out.obstacle_detected = front_dist < OBSTACLE_THRESHOLD
        msg_out.object_type      = object_type

        self.obstacle_pub.publish(msg_out)

        self.get_logger().info(
            f"Front={front_dist:.2f} m, Left={left_dist:.2f} m, Right={right_dist:.2f} m | "
            f"Obstacle={msg_out.obstacle_detected}, Type={object_type}",
            throttle_duration_sec=1
        )


def main(args=None):
    rclpy.init(args=args)
    detector = ObstacleDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()