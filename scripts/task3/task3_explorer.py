#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from com2009_team09_2026_modules.tb3_tools import quaternion_to_euler

import numpy as np
import random
from math import floor, degrees, atan2, pi, sqrt
import time

FRONT_ARC = list(range(0, 26)) + list(range(335, 360))
LEFT_ARC  = list(range(26, 91))
RIGHT_ARC = list(range(270, 335))
BACK_ARC  = list(range(150, 211))

MIN_VALID_RANGE  = 0.12
FORWARD_SPEED    = 0.22 # 0.26
SLOW_SPEED       = 0.12 # 0.10
TURN_SPEED       = 0.8 # 1.0
BACKUP_SPEED     = -0.15

FRONT_CLEAR      = 0.50 # 0.43
FRONT_VERY_CLOSE = 0.35 # 0.30
SIDE_CLOSE       = 0.22 # 0.18
BACK_CLEAR       = 0.25

GOAL_REACHED     = 0.40
GOAL_TIMEOUT     = 150
TOTAL_TIME       = 180.0


class Explorer(Node):

    def __init__(self):
        super().__init__("task3_explorer")

        self.shutdown   = False
        self.have_odom  = False
        self.have_scan  = False
        self.have_map   = False
        self.start_time = None
        self.vel_msg    = TwistStamped()

        self.x = self.y = self.theta_z = 0.0
        self.x0 = self.y0 = 0.0

        self.map_data     = None
        self.map_width    = self.map_height = 0
        self.map_res      = 0.05
        self.map_origin_x = self.map_origin_y = 0.0

        self.goal_x       = None
        self.goal_y       = None
        self.goal_age     = 0
        self.goal_blacklist = []  # blacklist timed-out goals

        self.visited_zones = set()

        self.front_distance = 999.0
        self.left_distance  = 999.0
        self.right_distance = 999.0
        self.back_distance  = 999.0

        self.prev_x = self.prev_y = 0.0
        self.stuck_count   = 0
        self.stuck_timer   = 0

        self.state           = "FIND_GOAL"
        self.blocked_counter = 0
        self.last_turn_left  = True
        self.backup_counter  = 0
        self.recover_counter = 0
        self.recover_dir     = 1
        self.recoveries      = 0

        self.vel_pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10)

        self.control_timer = self.create_timer(0.1, self.timer_callback)
        self.log_timer     = self.create_timer(2.0, self.log_callback)
        self.get_logger().info("Task3: Frontier explorer with blacklist started.")

    def set_cmd(self, v, w):
        self.vel_msg.twist.linear.x  = v
        self.vel_msg.twist.angular.z = w
        self.vel_pub.publish(self.vel_msg)

    def filter_ranges(self, ranges, indices):
        vals = [ranges[i] for i in indices
                if i < len(ranges)
                and ranges[i] > MIN_VALID_RANGE
                and not np.isinf(ranges[i])
                and not np.isnan(ranges[i])]
        return vals if vals else [float("inf")]

    def _zone(self):
        return (floor((self.x - self.x0)), floor((self.y - self.y0)))

    def _update_zones(self):
        z = self._zone()
        if z not in self.visited_zones:
            self.visited_zones.add(z)
            self.get_logger().info(f"ZONE {z} total={len(self.visited_zones)}")

    def _find_goal(self):
        if self.map_data is None:
            return False

        grid = np.array(self.map_data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))

        rx = int((self.x - self.map_origin_x) / self.map_res)
        ry = int((self.y - self.map_origin_y) / self.map_res)

        frontiers = []
        step = 3
        for gy in range(step, self.map_height - step, step):
            for gx in range(step, self.map_width - step, step):
                if grid[gy, gx] != 0:
                    continue
                patch = grid[max(0,gy-1):gy+2, max(0,gx-1):gx+2]
                if -1 not in patch:
                    continue
                dist = sqrt((gx-rx)**2 + (gy-ry)**2)
                if dist < 4:  # ignore frontiers too close
                    continue

                wx = gx * self.map_res + self.map_origin_x
                wy = gy * self.map_res + self.map_origin_y

                # Skip blacklisted locations
                blacklisted = False
                for bx, by in self.goal_blacklist:
                    if sqrt((wx-bx)**2 + (wy-by)**2) < 0.5:
                        blacklisted = True
                        break
                if blacklisted:
                    continue

                frontiers.append((dist, gx, gy, wx, wy))

        if not frontiers:
            # Clear blacklist and try again
            self.goal_blacklist = []
            self.get_logger().info("No frontiers — blacklist cleared")
            return False

        # Pick from farthest 30%
        frontiers.sort(reverse=True)
        pool = frontiers[:max(1, len(frontiers)//3)]
        _, bx, by, wx, wy = random.choice(pool)

        self.goal_x   = wx
        self.goal_y   = wy
        self.goal_age = 0
        self.get_logger().info(
            f"NEW GOAL: ({self.goal_x:.2f}, {self.goal_y:.2f}) "
            f"blacklist={len(self.goal_blacklist)}")
        return True

    def odom_callback(self, msg):
        pose = msg.pose.pose
        _, _, yaw = quaternion_to_euler(pose.orientation)
        self.x, self.y, self.theta_z = pose.position.x, pose.position.y, yaw
        if not self.have_odom:
            self.have_odom = True
            self.x0, self.y0 = self.x, self.y
            self.prev_x, self.prev_y = self.x, self.y
            self.visited_zones.add(self._zone())

    def scan_callback(self, msg):
        self.have_scan = True
        r = msg.ranges
        self.front_distance = float(np.min(self.filter_ranges(r, FRONT_ARC)))
        self.left_distance  = float(np.min(self.filter_ranges(r, LEFT_ARC)))
        self.right_distance = float(np.min(self.filter_ranges(r, RIGHT_ARC)))
        self.back_distance  = float(np.min(self.filter_ranges(r, BACK_ARC)))

    def map_callback(self, msg):
        self.have_map     = True
        self.map_data     = msg.data
        self.map_width    = msg.info.width
        self.map_height   = msg.info.height
        self.map_res      = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def _check_stuck(self):
        self.stuck_timer += 1
        if self.stuck_timer >= 15:
            d = sqrt((self.x-self.prev_x)**2 + (self.y-self.prev_y)**2)
            self.stuck_count = self.stuck_count + 1 if d < 0.03 else 0
            self.prev_x, self.prev_y = self.x, self.y
            self.stuck_timer = 0

    def timer_callback(self):
        if self.shutdown:
            return
        
        if not self.have_scan or not self.have_odom:
            return

        if self.start_time is None:
            self.start_time = time.time()

        if time.time() - self.start_time >= TOTAL_TIME:
            self.set_cmd(0.0, 0.0)
            return

        self._update_zones()

        front = self.front_distance
        left  = self.left_distance
        right = self.right_distance
        back  = self.back_distance

        self.last_turn_left = left >= right

        # BACKUP
        if self.state == "BACKUP":
            self.set_cmd(BACKUP_SPEED, 0.0)
            self.backup_counter -= 1
            if self.backup_counter <= 0:
                self.state           = "RECOVER"
                self.recover_counter = 20 + random.randint(0, 15)
                self.recover_dir     = 1 if left > right else -1
                if self.recoveries % 2 == 0:
                    self.recover_dir *= -1
                self.recoveries += 1
                self.stuck_count = 0
                self.stuck_timer = 0
                self.prev_x, self.prev_y = self.x, self.y
            return

        # RECOVER 
        if self.state == "RECOVER":
            self.set_cmd(0.0, TURN_SPEED * self.recover_dir)
            self.recover_counter -= 1
            if self.recover_counter <= 0:
                self.state  = "FIND_GOAL"
                self.goal_x = None
                self.goal_y = None
            return

        #  Stuck check 
        self._check_stuck()
        if self.stuck_count >= 3:
            if back > BACK_CLEAR:
                self.state          = "BACKUP"
                self.backup_counter = 15
            else:
                self.state           = "RECOVER"
                self.recover_counter = 20 + random.randint(0, 20)
                self.recover_dir     = random.choice([1, -1])
                self.recoveries     += 1
            self.stuck_count = 0
            self.goal_x      = None
            return

        #  Goal timeout 
        if self.goal_x is not None:
            self.goal_age += 1
            if self.goal_age > GOAL_TIMEOUT:
                self.get_logger().info("Goal timed out — blacklisting")
                self.goal_blacklist.append((self.goal_x, self.goal_y))
                if len(self.goal_blacklist) > 20:
                    self.goal_blacklist.pop(0)
                self.goal_x = None
                self.goal_y = None
                self.state  = "FIND_GOAL"

        # FIND_GOAL 
        if self.state == "FIND_GOAL" or self.goal_x is None:
            if self.have_map and self._find_goal():
                self.state = "GOTO_GOAL"
            else:
                self.state = "WANDER"

        # AVOID
        if front < FRONT_CLEAR:
            self.blocked_counter += 1
            if self.blocked_counter > 20:
                self.last_turn_left  = not self.last_turn_left
                self.blocked_counter = 0

            if front < FRONT_VERY_CLOSE:
                self.set_cmd(0.0,
                    TURN_SPEED if self.last_turn_left else -TURN_SPEED)
            else:
                self.set_cmd(SLOW_SPEED,
                    TURN_SPEED if self.last_turn_left else -TURN_SPEED)
            return

        self.blocked_counter = 0

        # GOTO_GOAL 
        if self.state == "GOTO_GOAL" and self.goal_x is not None:
            dx   = self.goal_x - self.x
            dy   = self.goal_y - self.y
            dist = sqrt(dx**2 + dy**2)

            if dist < GOAL_REACHED:
                self.get_logger().info("GOAL REACHED")
                self.goal_x = None
                self.goal_y = None
                self.state  = "FIND_GOAL"
                return

            target = atan2(dy, dx)
            err    = target - self.theta_z
            while err > pi:  err -= 2*pi
            while err < -pi: err += 2*pi

            if left < SIDE_CLOSE:
                self.set_cmd(SLOW_SPEED, -0.6)
            elif right < SIDE_CLOSE:
                self.set_cmd(SLOW_SPEED, 0.6)
            elif abs(err) > 0.8:
                w = max(min(1.5 * err, 1.2), -1.2)
                self.set_cmd(FORWARD_SPEED, w)
            else:
                w = max(min(1.0 * err, 0.7), -0.7)
                self.set_cmd(FORWARD_SPEED, w)
            return

        # WANDER 
        if left < SIDE_CLOSE:
            self.set_cmd(FORWARD_SPEED, -0.4)
        elif right < SIDE_CLOSE:
            self.set_cmd(FORWARD_SPEED, 0.4)
        else:
            w = max(min(0.4 * (left - right), 0.3), -0.3)
            self.set_cmd(FORWARD_SPEED, w)

        if random.random() < 0.05 and self.have_map:
            self.state = "FIND_GOAL"

    def log_callback(self):
        if not self.have_scan or not self.have_odom:
            return
        t = time.time() - self.start_time if self.start_time else 0.0
        z = self._zone()
        g = (f"({self.goal_x:.1f},{self.goal_y:.1f})"
             if self.goal_x is not None else "None")
        self.get_logger().info(
            f"t={t:.0f}s | {self.state} | zone={z} "
            f"vis={len(self.visited_zones)} | goal={g} age={self.goal_age} | "
            f"F={self.front_distance:.2f} L={self.left_distance:.2f} "
            f"R={self.right_distance:.2f} | rec={self.recoveries} "
            f"stuck={self.stuck_count} bl={len(self.goal_blacklist)}")

    def on_shutdown(self):
        self.shutdown = True
        stop_msg = TwistStamped()
        for _ in range(10):
            try:
                self.vel_pub.publish(stop_msg)
            except Exception:
                break
            time.sleep(0.05)
                
def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
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