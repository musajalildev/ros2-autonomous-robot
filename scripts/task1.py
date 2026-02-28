#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from math import pi, atan2, degrees

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
        if not self.first_message:
            return

        radius = 0.5 # meters
        linear_velocity = 0.1047 # meters per second [m/s]
        angular_velocity = linear_velocity / radius # radians per second [rad/s]
        
        angle_change = self.theta_z - self.theta_zref

        if angle_change > pi:
            angle_change -= 2 * pi
        elif angle_change < -pi:
            angle_change += 2 * pi

        self.angle_travelled += abs(angle_change)
        self.theta_zref = self.theta_z

        if self.loop == 1:
            # First loop: anticlockwise
            if self.angle_travelled < 2 * pi:
                self.vel_msg.twist.linear.x = linear_velocity
                self.vel_msg.twist.angular.z = angular_velocity
            else:
                self.loop = 2
                self.angle_travelled = 0.0

        elif self.loop == 2:
            # Second loop: clockwise
            if self.angle_travelled < 2 * pi:
                self.vel_msg.twist.linear.x = linear_velocity
                self.vel_msg.twist.angular.z = -angular_velocity
            else:
                # Finished figure-of-eight
                self.vel_msg.twist.linear.x = 0.0
                self.vel_msg.twist.angular.z = 0.0

        self.vel_pub.publish(self.vel_msg)

    def log_callback(self):
        if not self.first_message:
            return

        x_rel = self.x - self.x0
        y_rel = self.y - self.y0
        theta_rel = self.theta_z - self.theta0

        self.get_logger().info(
            f"x={x_rel:.2f} [m], "
            f"y={y_rel:.2f} [m], "
            f"yaw={degrees(theta_rel):.1f} [degrees]."
        )
        
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
    