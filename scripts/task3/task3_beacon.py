#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

SNAP_PATH = os.path.expanduser(
  '~/ros2_ws/src/com2009_team09_2026/snaps/target_beacon.jpg'
)

# HSV colour ranges for each beacon colour
COLOUR_RANGES = {
  'yellow': ([20, 100, 100], [35, 255, 255]),
  'green': ([36, 50, 50], [89, 255, 255]),
  'blue': ([90, 50, 50], [128, 255, 255]),
  'red': ([0, 120, 70], [10, 255, 255]),
}

# Minimum contour area in pixels to count as a real detection
MIN_CONTOUR_AREA = 100

class BeaconSearch(Node):

  def __init__(self):
    super().__init__("task3_beacon")

    self.shutdown = False

    self.declare_parameter('target_beacon', 'blue')
    self.colour = (
      self.get_parameter('target_beacon')
      .get_parameter_value()
      .string_value.lower()
    )

    # Required log message - must appear within 10 seconds
    self.get_logger().info(f"TARGET BEACON: Searching for {self.colour}.")

    self.bridge = CvBridge()
    self.best_score = 0 # track best sighting for progressive saving
    self.saved = False

    # Ensure snaps directory exists
    os.makedirs(os.path.dirname(SNAP_PATH), exist_ok=True)

    self.image_sub = self.create_subscription(
      Image,
      'camera/image_raw',
      self.image_callback,
      10
    )

  def image_callback(self, msg: Image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception:
      return
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lo, hi = COLOUR_RANGES.get(self.colour, COLOUR_RANGES['blue'])
    mask = cv2.inRange(
      hsv,
      np.array(lo, dtype=np.uint8),
      np.array(hi, dtype=np.uint8),
    )

    # Red wraps around the HSV hue circle - add the upper red band
    if self.colour == 'red':
      mask2 = cv2.inRange(
        hsv,
        np.array([160, 120, 70], dtype=np.uint8),
        np.array([180, 255, 255], dtype=np.uint8),
      )
      mask = cv2.bitwise_or(mask, mask2)

    # Find contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_CONTOUR_AREA:
      return
 
    # Score: area * height fraction
    # Favours close-up views where the full beacon is visible (C3-C5 marks)
    _, _, bbox_w, bbox_h = cv2.boundingRect(largest)
    img_h = cv_image.shape[0]
    height_frac = bbox_h / img_h
    score = area * height_frac

    # Only save if this is a BETTER view than what we already have
    if score > self.best_score:
      self.best_score = score
      # Save the RAW image (no filtering applied)
      cv2.imwrite(SNAP_PATH, cv_image)

      if not self.saved:
        self.saved = True
        self.get_logger().info(
            f"BEACON CAPTURED: {self.colour} beacon saved to {SNAP_PATH}"
        )
      else:
        self.get_logger().info(
            f"BEACON UPDATED: better image saved "
            f"(score={score:.0f}, area={area:.0f}, h_frac={height_frac:.2f})"
        )

  def on_shutdown(self):
    if self.saved:
      self.get_logger().info("Beacon node shutting down - image was saved.")
    else:
      self.get_logger().warn("Beacon node shutting down - NO beacon image was captured!")
    self.shutdown = True

def main(args=None):
  rclpy.init(
    args=args,
    signal_handler_options=SignalHandlerOptions.NO
  )
  node = BeaconSearch()
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