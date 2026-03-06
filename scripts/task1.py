#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from math import pi, atan2, degrees, sin, cos 

class Task1(Node):

    def __init__(self):
        super().__init__("velocity_control")

        self.first_message = False
        self.shutdown = False
        self.loop = 1 # 1 = first circle, 2 = second circle

        self.vel_msg = TwistStamped()

        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.theta_zref = 0.0
        self.angle_travelled = 0.0

        self.x0 = 0.0; self.y0 = 0.0; self.theta0 = 0

        self.cx = None
        self.cy = None
        self.phi_ref = 0.0
        self.phi_travelled = 0.0

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

    def quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw # in radians

    def on_shutdown(self):
        self.get_logger().info("Stopping the robot...")
        self.vel_pub.publish(TwistStamped())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
            pose = msg_data.pose.pose

            yaw = self.quaternion_to_euler(pose.orientation)

            self.x = pose.position.x
            self.y = pose.position.y
            self.theta_z = yaw

            if not self.first_message:
                self.first_message = True
                self.x0 = self.x
                self.y0 = self.y
                self.theta0 = self.theta_z

    def timer_callback(self):
        radius = 0.5
        linear_velocity = 0.1047
        angular_velocity = linear_velocity / radius

        # Initialise the first circle centre once we have odom
        if self.cx is None:
            # loop 1 is CCW
            self.set_circle_center(clockwise=False, radius=radius)

        # Track progress around the circle using position angle about centre
        phi = atan2(self.y - self.cy, self.x - self.cx)
        dphi = self.wrap_to_pi(phi - self.phi_ref)
        self.phi_travelled += abs(dphi)
        self.phi_ref = phi

        if self.loop == 1:
            # CCW
            self.vel_msg.twist.linear.x = linear_velocity
            self.vel_msg.twist.angular.z = +angular_velocity

            if self.phi_travelled >= 2 * pi:
                self.loop = 2
                # recompute centre for the second circle (CW) from the CURRENT pose
                self.set_circle_center(clockwise=True, radius=radius)

        elif self.loop == 2:
            # CW
            self.vel_msg.twist.linear.x = linear_velocity
            self.vel_msg.twist.angular.z = -angular_velocity

            if self.phi_travelled >= 2 * pi:
                self.vel_msg.twist.linear.x = 0.0
                self.vel_msg.twist.angular.z = 0.0

        self.vel_pub.publish(self.vel_msg)
    
    def wrap_to_pi(self, a: float) -> float:
        # robust wrap
        return atan2(sin(a), cos(a))

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
    rclpy.init(args=args)
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


# state = 1
# vel = TwistStamped()

# rclpy.init(args=None)
# node = rclpy.create_node("basic_velocity_control")
# vel_pub = node.create_publisher(TwistStamped, "cmd_vel", 10)

# timestamp = node.get_clock().now().nanoseconds

# while rclpy.ok():
#     time_now = node.get_clock().now().nanoseconds
#     elapsed_time = (time_now - timestamp) * 1e-9
#     if state == 1: 
#         if elapsed_time < 30:
#             vel.twist.linear.x = 0.1047
#             vel.twist.angular.z = 0.2094
#         else:
#             # vel.twist.linear.x = 0.0
#             vel.twist.angular.z = 0.0
#             state = 2
#             timestamp = node.get_clock().now().nanoseconds
#     elif state == 2:
#         if elapsed_time < 30:
#             vel.twist.linear.x = 0.1047
#             vel.twist.angular.z = -0.2094
#         else:
#             vel.twist.linear.x = 0.0
#             vel.twist.angular.z = 0.0 
#             break

#     node.get_logger().info(
#         f"\n[State = {state}] Publishing velocities:\n"
#         f"  - linear.x: {vel.twist.linear.x:.2f} [m/s]\n"
#         f"  - angular.z: {vel.twist.angular.z:.2f} [rad/s].",
#         throttle_duration_sec=1,
#     )
#     vel_pub.publish(vel)
    
#     try:
#         rclpy.spin_once(node, timeout_sec=0.1)
#         time.sleep(0.1) # 10Hz loop rate
#     except KeyboardInterrupt:
#         print("Ctrl+C detected. Shutting down.")
#         break

# node.destroy_node()
    