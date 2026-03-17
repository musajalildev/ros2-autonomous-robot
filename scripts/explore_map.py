#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from com2009_team09_2026.msg import ObstacleInfo
from com2009_team09_2026_modules.tb3_tools import quaternion_to_euler

from math import degrees

class MapExplorer(Node):

    def __init__(self):
        super().__init__("map_explorer")

        self.shutdown = False
        self.have_obstacle_info = False
        self.have_odom = False

        self.vel_msg = TwistStamped()
        self.latest_obstacle = None

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        # Progress tracking
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.progress_timer_count = 0
        self.no_progress_count = 0

        # Simple state machine
        self.state = "EXPLORE" # States: EXPLORE, AVOID, RECOVER
        self.blocked_counter = 0
        self.last_turn_left = True
        self.recover_counter = 0

        # Tunable parameters
        self.forward_speed = 0.23
        self.slow_speed = 0.06
        self.turn_speed = 0.9
        self.recover_turn_speed = 1.2

        self.front_clear_threshold = 0.43
        self.front_very_close_threshold = 0.30
        self.side_close_threshold = 0.18

        # Publisher
        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        )

        # Subscriber
        self.obstacle_sub = self.create_subscription(
            msg_type=ObstacleInfo,
            topic="/obstacle_info",
            callback=self.obstacle_callback,
            qos_profile=10,
        )

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        # Timers
        self.control_timer = self.create_timer(
            timer_period_sec=0.1,   # 10 Hz
            callback=self.timer_callback,
        )

        self.log_timer = self.create_timer(
            timer_period_sec=1.0,   # 1 Hz
            callback=self.log_callback,
        )

        self.get_logger().info("Explore map node started.")

    def on_shutdown(self):
        self.get_logger().info("Stopping the robot...")
        stop_msg = TwistStamped()
        self.vel_pub.publish(stop_msg)
        self.shutdown = True

    def obstacle_callback(self, msg: ObstacleInfo):
        self.latest_obstacle = msg
        self.have_obstacle_info = True

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        _, _, yaw = quaternion_to_euler(pose.orientation)

        self.x = pose.position.x
        self.y = pose.position.y
        self.theta_z = yaw

        if not self.have_odom:
            self.have_odom = True
            self.prev_x = self.x
            self.prev_y = self.y

    def set_cmd(self, linear_x: float, angular_z: float):
        self.vel_msg.twist.linear.x = linear_x
        self.vel_msg.twist.angular.z = angular_z
        self.vel_pub.publish(self.vel_msg)

    def update_progress_check(self):
        self.progress_timer_count += 1

        if self.progress_timer_count >= 10:
            dx = self.x - self.prev_x
            dy = self.y - self.prev_y
            dist_moved = (dx**2 + dy**2)**0.5

            if dist_moved < 0.03:
                self.no_progress_count += 1
            else:
                self.no_progress_count = 0

            self.prev_x = self.x
            self.prev_y = self.y
            self.progress_timer_count = 0

    def timer_callback(self):
        if not self.have_obstacle_info or not self.have_odom:
            return
        
        if self.state != "RECOVER":
            self.update_progress_check()

        front = self.latest_obstacle.front_distance
        left = self.latest_obstacle.left_distance
        right = self.latest_obstacle.right_distance
        obstacle_detected = self.latest_obstacle.obstacle_detected

        self.last_turn_left = True if left >= right else False

        # Recovery trigger
        if self.state != "RECOVER" and self.no_progress_count >= 2:
            self.state = "RECOVER"
            self.recover_counter = 12 # about 1.2 seconds
            self.no_progress_count = 0

        if self.state == "RECOVER":
            if self.last_turn_left:
                self.set_cmd(0.0, self.recover_turn_speed)
            else:
                self.set_cmd(0.0, -self.recover_turn_speed)

            self.recover_counter -= 1
            if self.recover_counter <= 0:
                self.state = "EXPLORE"
                self.blocked_counter = 0
                self.no_progress_count = 0
                self.progress_timer_count = 0
                self.prev_x = self.x
                self.prev_y = self.y
            return 

        # If front is blocked or very close, turn away from closer side
        if obstacle_detected or front < self.front_clear_threshold:
            self.state = "AVOID"
            self.blocked_counter += 1

            if self.blocked_counter > 15:
                self.last_turn_left = not self.last_turn_left
                self.blocked_counter = 0

            # If very close to obstacle, turn in place.
            if front < self.front_very_close_threshold:
                if self.last_turn_left:
                    self.set_cmd(0.0, self.turn_speed)
                else:
                    self.set_cmd(0.0, -self.turn_speed)
                return
            
            # Otherwise, move forward slowly while turning.
            if self.last_turn_left:
                self.set_cmd(self.slow_speed, self.turn_speed)
            else:
                self.set_cmd(self.slow_speed, -self.turn_speed)
            return

        # Otherwise, continue exploring
        self.state = "EXPLORE"
        self.blocked_counter = 0

        angular_z = 0.0

        if left < self.side_close_threshold:
            angular_z = -0.45
        elif right < self.side_close_threshold:
            angular_z = 0.45
        else:
            # Very small bias to the more open side
            error = left - right
            angular_z = max(min(0.35 * error, 0.25), -0.25)

        self.set_cmd(self.forward_speed, angular_z)

    def log_callback(self):
        if not self.have_obstacle_info or not self.have_odom:
            return

        self.get_logger().info(
            f"State={self.state} | "
            f"x={self.x:.2f} m, y={self.y:.2f} m, yaw={degrees(self.theta_z):.1f} deg | "
            f"BlockedCount={self.blocked_counter}, NoProgress={self.no_progress_count}"
        )


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MapExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"{node.get_name()} received a shutdown request (Ctrl+C).")
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()