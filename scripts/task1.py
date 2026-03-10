#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from com2009_team09_2026_modules.tb3_tools import quaternion_to_euler

from math import pi, atan2, sin, cos, degrees, sqrt

class Task1(Node):

    def __init__(self):
        super().__init__("velocity_control")

        self.first_message = False
        self.shutdown = False
        self.loop = 1 # 1 = first circle, 2 = second circle

        self.vel_msg = TwistStamped()

        # Current pose from odom
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        # Start pose
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0

        # Circle tracking
        self.cx = None
        self.cy = None
        self.phi_ref = 0.0
        self.phi_travelled = 0.0
        
        self.start_tolerance = 0.07 # [m] how close to start pose to consider "starting"

        # Publisher
        self.vel_pub = self.create_publisher(
            msg_type=TwistStamped,
            topic="cmd_vel",
            qos_profile=10,
        )

        # Subscriber
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.control_timer = self.create_timer(
            timer_period_sec=0.1, # 10Hz control loop
            callback=self.timer_callback,
        )

        self.log_timer = self.create_timer(
            timer_period_sec=1.0, # 1Hz logging
            callback=self.log_callback,
        )

    def on_shutdown(self):
        self.get_logger().info("Stopping the robot...")
        self.vel_pub.publish(TwistStamped())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
            pose = msg_data.pose.pose
            _, _, yaw = quaternion_to_euler(pose.orientation)

            self.x = pose.position.x
            self.y = pose.position.y
            self.theta_z = yaw

            if not self.first_message:
                self.first_message = True
                self.x0 = self.x
                self.y0 = self.y
                self.theta0 = self.theta_z

    def timer_callback(self):
        if not self.first_message:
            return

        radius = 0.5
        linear_velocity = 0.11
        angular_velocity = linear_velocity / radius

        # Initialise the first circle centre once we have odom
        if self.cx is None:
            self.set_circle_center(clockwise=False, radius=radius)

        # Track progress around the circle using position angle about centre
        phi = atan2(self.y - self.cy, self.x - self.cx)
        dphi = self.wrap_to_pi(phi - self.phi_ref)
        self.phi_travelled += abs(dphi)
        self.phi_ref = phi

        # Distance from start point
        dx0 = self.x - self.x0
        dy0 = self.y - self.y0
        distance_to_start = sqrt(dx0**2 + dy0**2)

        if self.loop == 1:
            # First loop: Anti-clockwise
            self.vel_msg.twist.linear.x = linear_velocity
            self.vel_msg.twist.angular.z = +angular_velocity

            if self.phi_travelled >= 1.5 * pi and distance_to_start < self.start_tolerance:
                self.loop = 2
                self.set_circle_center(clockwise=True, radius=radius)

        elif self.loop == 2:
            # Second loop: Clockwise
            self.vel_msg.twist.linear.x = linear_velocity
            self.vel_msg.twist.angular.z = -angular_velocity

            if self.phi_travelled >= 1.5 * pi and distance_to_start < self.start_tolerance:
                self.vel_msg.twist.linear.x = 0.0
                self.vel_msg.twist.angular.z = 0.0

        self.vel_pub.publish(self.vel_msg)

    def log_callback(self):
        if not self.first_message:
            return

        x_rel = self.x - self.x0
        y_rel = self.y - self.y0
        theta_rel = self.wrap_to_pi(self.theta_z - self.theta0)

        self.get_logger().info(
            f"{self.loop}: phi_travelled {self.phi_travelled}"
            f"x={x_rel:.2f} [m], "
            f"y={y_rel:.2f} [m], "
            f"yaw={degrees(theta_rel):.1f} [degrees]."
        )
    
    def wrap_to_pi(self, angle: float) -> float:
        return atan2(sin(angle), cos(angle))

    def set_circle_center(self, clockwise: bool, radius: float):
        # compute center from current pose
        if clockwise:
            self.cx = self.x + radius * sin(self.theta_z)
            self.cy = self.y - radius * cos(self.theta_z)
        else:
            self.cx = self.x - radius * sin(self.theta_z)
            self.cy = self.y + radius * cos(self.theta_z)

        self.phi_ref = atan2(self.y - self.cy, self.x - self.cx)
        self.phi_travelled = 0.0
        
def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Task1()
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