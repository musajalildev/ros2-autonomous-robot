#!/usr/bin/env python3
"""
Task 1: Figure-of-Eight Motion Profile
COM2009 Assignment #2

Hybrid control:
  - Yaw accumulation detects when the robot has rotated ~300 degrees
    (i.e. it's on the final approach back to the start)
  - Then switches to distance-from-start to stop precisely at the
    crossover point, giving accurate loop centre and stop position.

Arena layout:
  - Robot starts at crossover point facing +x direction
  - Red beacon is 0.5m to the LEFT  (Loop 1, anti-clockwise)
  - Blue beacon is 0.5m to the RIGHT (Loop 2, clockwise)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import math
from tf_transformations import euler_from_quaternion


class Task1Node(Node):

    def __init__(self):
        super().__init__('velocity_control')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Odometry state
        self.odom_received = False
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Motion parameters
        self.radius = 0.5
        self.linear_speed = 0.26   # m/s
        self.angular_speed = self.linear_speed / self.radius  # 0.52 rad/s

        # Yaw tracking
        self.yaw_accumulated = 0.0
        self.prev_yaw = 0.0

        # After this much rotation, switch from yaw-mode to
        # distance-from-start mode for precise stopping
        self.approach_rad = math.radians(300)  # 300° — on final approach
        self.return_threshold = 0.06           # stop within 6 cm of start

        # Phase:
        # 'init' → 'loop1_yaw' → 'loop1_home'
        #        → 'loop2_yaw' → 'loop2_home'
        #        → 'done'
        self.phase = 'init'

        self.create_timer(0.1, self.control_callback)
        self.create_timer(1.0, self.log_callback)

        self.get_logger().info('Task 1 node started. Waiting for first odometry message...')

    # ── Odometry ─────────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

        if not self.odom_received:
            self.initial_x = pos.x
            self.initial_y = pos.y
            self.initial_yaw = yaw
            self.prev_yaw = yaw
            self.odom_received = True
            self.get_logger().info('Initial odometry captured. Starting motion...')

        self.current_x = pos.x
        self.current_y = pos.y
        self.current_yaw = yaw

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _relative_pose(self):
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        dyaw_rad = math.atan2(
            math.sin(self.current_yaw - self.initial_yaw),
            math.cos(self.current_yaw - self.initial_yaw)
        )
        return dx, dy, math.degrees(dyaw_rad)

    def _distance_from_start(self):
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        return math.sqrt(dx * dx + dy * dy)

    def _update_yaw_accumulator(self, direction: float):
        delta = self.current_yaw - self.prev_yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))
        self.yaw_accumulated += direction * delta
        self.prev_yaw = self.current_yaw
        return self.yaw_accumulated

    def _start_loop(self, loop_name: str):
        self.yaw_accumulated = 0.0
        self.prev_yaw = self.current_yaw
        self.get_logger().info(f'Starting {loop_name}...')

    # ── 1 Hz logger ──────────────────────────────────────────────────────────

    def log_callback(self):
        if not self.odom_received:
            return
        x, y, yaw = self._relative_pose()
        self.get_logger().info(
            f'x={x:.2f} [m], y={y:.2f} [m], yaw={yaw:.1f} [degrees].'
        )

    # ── 10 Hz control loop ────────────────────────────────────────────────────

    def control_callback(self):
        if not self.odom_received:
            return

        if self.phase == 'init':
            self.phase = 'loop1_yaw'
            self._start_loop('Loop 1 (anti-clockwise)')

        elif self.phase == 'loop1_yaw':
            # Drive anti-clockwise until 300° rotated
            rotated = self._update_yaw_accumulator(direction=+1.0)
            self._publish_velocity(self.linear_speed, +self.angular_speed)
            if rotated >= self.approach_rad:
                self.phase = 'loop1_home'
                self.get_logger().info('Loop 1 final approach...')

        elif self.phase == 'loop1_home':
            # Keep driving but stop as soon as we're back near the start
            self._update_yaw_accumulator(direction=+1.0)
            if self._distance_from_start() > self.return_threshold:
                self._publish_velocity(self.linear_speed, +self.angular_speed)
            else:
                self._publish_velocity(0.0, 0.0)
                self.phase = 'loop2_yaw'
                self._start_loop('Loop 2 (clockwise)')

        elif self.phase == 'loop2_yaw':
            # Drive clockwise until 300° rotated
            rotated = self._update_yaw_accumulator(direction=-1.0)
            self._publish_velocity(self.linear_speed, -self.angular_speed)
            if rotated >= self.approach_rad:
                self.phase = 'loop2_home'
                self.get_logger().info('Loop 2 final approach...')

        elif self.phase == 'loop2_home':
            # Keep driving but stop as soon as we're back near the start
            self._update_yaw_accumulator(direction=-1.0)
            if self._distance_from_start() > self.return_threshold:
                self._publish_velocity(self.linear_speed, -self.angular_speed)
            else:
                self._publish_velocity(0.0, 0.0)
                self.phase = 'done'
                self.get_logger().info('Figure-of-eight complete. Robot stopped.')

        elif self.phase == 'done':
            self._publish_velocity(0.0, 0.0)

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish_velocity(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Task1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    stop_msg = TwistStamped()
    stop_msg.header.stamp = node.get_clock().now().to_msg()
    node.cmd_pub.publish(stop_msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()