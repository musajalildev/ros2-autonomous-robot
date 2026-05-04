#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs import LaserScan
from com2009_team09_2026_modules.tb3_tools import quaternion_to_euler

import numpy as np
from math import floor, degrees
import time

# LiDAR arc indices

FRONT_ARC = list(range(0, 26)) + list(range(335, 360))
LEFT_ARC = list(range(26, 91))
RIGHT_ARC = list(range(270, 335))

MIN_VALID_RANGE = 0.12

# TUNABLE PARAMETERS

FORWARD_SPEED = 0.23
SLOW_SPEED = 0.06
TURN_SPEED = 0.9
RECOVER_TURN_SPEED = 1.2

FRONT_CLEAR_THRESHOLD = 0.43
FRONT_VERY_CLOSE_THRESHOLD = 0.30
SIDE_CLOSE_THRESHOLD = 0.18

ZONE_SIZE = 1.0

TOTAL_TIME = 178.0 # stop robot (2s before 180)

class Explorer(Node):

  def __init__(self):
    super().__init__("task3_explorer")

    self.shutdown = False
    self.first_odom = False
    self.first_scan = False
    self.start_time = None

    self.vel_msg = TwistStamped

    # Odometry
    self.x = 0.0
    self.y = 0.0
    self.theta_z = 0.0

    # Start pose
    self.x0 = 0.0
    self.y0 = 0.0

    # Zone tracking
    self.visited_zones = set()
    self.start_zone = None

    # LiDAR distance
    self.front_distance = 999.0
    self.left_distance = 999.0
    self.right_distance = 999.0

    # Progress tracking
    self.prev_x = 0.0
    self.prev_y = 0.0
    self.progress_timer_count = 0
    self.no_progress_count = 0

    # State machine
    self.state = "EXPLORE"
    self.blocked_counter = 0
    self.last_turn_left = True
    self.recover_counter = 0

    # Publishers
    self.vel_pub = self.create_publisher(
      msg_type=TwistStamped,
      topic="cmd_vel",
      qos_profile=10
    )

    # Subscribers
    self.odom_sub = self.create_subscription(
      msg_type=Odometry,
      topic="/odom",
      callback=self.odom_callback,
      qos_profile=10
    )
    self.scan_sub = self.create_subscription(
      msg_type=LaserScan,
      topic="/scan",
      callback=self.scan_callback,
      qos_profile=10
    )

    # Timers
    self.control_timer = self.create_timer(
      timer_period_sec=0.1, # 10 Hz
      callback=self.timer_callback
    )
    self.log_timer = self.create_timer(
      timer_period_sec=1.0, # 1 Hz
      callback=self.log_callback
    )

    self.get_logger().info("Task 3: explorer node started.")


  # HELPERS

  def set_cmd(self, linear_x: float, angular_z: float):
    self.vel_msg.twist.linear.x = linear_x
    self.vel_msg.twist.angular.z = angular_z
    self.vel_pub.publish(self.vel_msg)

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
  

  # ZONE TRACKING

  def _current_zone(self):
    rx = self.x - self.x0
    ry = self.y - self.y0
    return (floor(rx / ZONE_SIZE), floor(ry / ZONE_SIZE))

  def _update_zones(self):
    zone = self._current_zone()
    if zone not in self.visited_zones:
      self.visited_zones.add(zone)
      self.get_logger().info(f"ZONE ENTERED: {zone} - total visited: {len(self.visited_zones)}")


  # PROGRESS TRACKING

  def update_progress_check(self):
    self.progress_timer_count += 1

    if self.progress_timer_count >= 10:
      dx = self.x - self.prev_x
      dy = self.y - self.prev_y
      dist_moved = (dx**2 + dy**2) ** 0.5

      if dist_moved < 0.03:
        self.no_progress_count += 1
      else:
        self.no_progress_count = 0

      self.prev_x = self.x
      self.prev_y = self.y
      self.progress_timer_count = 0


  # SENSOR CALLBACKS

  def odom_callback(self, msg: Odometry):
    pose = msg.pose.pose
    _, _, yaw = quaternion_to_euler(pose.orientation)

    self.x = pose.position.x
    self.y = pose.position.y
    self.theta_z = yaw

    if not self.have_odom:
      self.have_odom = True
      self.x0 = self.x
      self.y0 = self.y
      self.prev_x = self.x
      self.prev_y = self.y
      self.start_zone = self._current_zone()
      self.visited_zones.add(self.start_zone)

  def scan_callback(self, msg: LaserScan):
    self.have_scan = True
    ranges = msg.ranges

    front_vals = self.filter_ranges(ranges, FRONT_ARC)
    left_vals = self.filter_ranges(ranges, LEFT_ARC)
    right_vals = self.filter_ranges(ranges, RIGHT_ARC)

    self.front_distance = float(np.min(front_vals))
    self.left_distance = float(np.min(left_vals))
    self.right_distance = float(np.min(right_vals))

  
  # MAIN CONTROL LOOP - 10 Hz

  def timer_callback(self):
    if not self.have_scan or not self.have_odom:
      return

    # Start clock
    if self.start_time is None:
      self.start_time = time.time()

    elapsed = time.time() - self.start_time

    # Time's up
    if elapsed >= TOTAL_TIME:
      self.set_cmd(0.0, 0.0)
      return
    
    # Update zone tracking
    self._update_zones()

    # Progress check
    if self.state != "RECOVER":
      self.update_progress_check()

    front = self.front_distance
    left = self.left_distance
    right = self.right_distance

    self.last_turn_left = True if left >= right else False

    # Recovery trigger
    if self.state != "RECOVER" and self.no_progress_count >= 2:
      self.state = "RECOVER"
      self.recover_counter = 12
      self.no_progress_count = 0

    # STATE: RECOVER
    if self.state == "RECOVER":
      if self.last_turn_left:
        self.set_cmd(0.0, RECOVER_TURN_SPEED)
      else:
        self.set_cmd(0.0, -RECOVER_TURN_SPEED)
      
      self.recover_counter -= 1
      if self.recover_counter <= 0:
        self.state = "EXPLORE"
        self.blocked_counter = 0
        self.no_progress_count = 0
        self.progress_timer_count = 0
        self.prev_x = self.x
        self.prev_y = self.y
      return
    
    # STATE: AVOID
    if front < FRONT_CLEAR_THRESHOLD:
      self.state = "AVOID"
      self.blocked_counter += 1

      # Flip turn direction if stuck turning one way too long
      if self.blocked_counter > 15:
        self.last_turn_left = not self.last_turn_left
        self.blocked_counter = 0

      # Very close - turn in place
      if front < FRONT_VERY_CLOSE_THRESHOLD:
        if self.last_turn_left:
          self.set_cmd(0.0, TURN_SPEED)
        else: 
          self.set_cmd(0.0, -TURN_SPEED)
        return
      
      # Close but not critical - slow forward + turn
      if self.last_turn_left:
        self.set_cmd(SLOW_SPEED, TURN_SPEED)
      else:
        self.set_cmd(SLOW_SPEED, -TURN_SPEED)
      return
    
    # STATE: EXPLORE
    self.state = "EXPLORE"
    self.blocked_counter = 0

    angular_z = 0.0

    # Push away from close side walls
    if left < SIDE_CLOSE_THRESHOLD:
      angular_z = -0.45
    elif right < SIDE_CLOSE_THRESHOLD:
      angular_z = 0.45
    else:
      error = left - right
      angular_z = max(min(0.35 * error, 0.25), -0.25)

    self.set_cmd(FORWARD_SPEED, angular_z)

  
  # LOGGING

  def log_callback(self):
    if not self.have_scan or not self.have_scan:
      return 
    
    elapsed = 0.0
    if self.start_time is not None:
      elapsed = time.time() - self.start_time

    zone = self._current_zone()
    self.get_logger().info(
      f"time={elapsed:.0f}s | State={self.state} | "
      f"zone={zone} | visited={len(self.visited_zones)} | "
      f"x={self.x:.2f} y={self.y:.2f} yaw={degrees(self.theta_z):.1f} deg | "
      f"Front={self.front_distance:.2f} Left={self.left_distance:.2f} "
      f"Right={self.right_distance:.2f} | "
      f"Blocked={self.blocked_counter} NoProgress={self.no_progress_count}"
    )


  # SHUTDOWN
  def on_shutdown(self):
    self.get_logger().info("Explorer node shutting down - stopping robot.")    
    self.vel_pub.publish(TwistStamped())
    self.shutdown = True

def main(args=None):
  rclpy.init(
    args=args,
    signal_handler_options=SignalHandlerOptions.NO
  )
  node = Explorer()
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