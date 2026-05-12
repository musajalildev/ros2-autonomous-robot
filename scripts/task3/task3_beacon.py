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

# Wide HSV colour ranges tuned for real-world lighting conditions
COLOUR_RANGES = {
    'yellow': ([20,  80,  80], [35, 255, 255]),
    'green':  ([36,  40,  40], [89, 255, 255]),
    'blue':   ([90,  40,  40], [130, 255, 255]),
    'red':    ([0,   150, 150], [10, 255, 255]),
}

# Second red range (wraps around hue circle)
RED_UPPER = ([165, 150, 150], [180, 255, 255]) 

# Minimum blob area to count as real detection
MIN_CONTOUR_AREA = 300

# Width fraction of image the beacon must cover for a "good" image
GOOD_WIDTH_FRAC = 0.15


class BeaconSearch(Node):

    def __init__(self):
        super().__init__("task3_beacon")

        self.shutdown = False

        self.declare_parameter('target_colour', 'blue')
        self.colour = (
            self.get_parameter('target_colour')
            .get_parameter_value()
            .string_value.lower()
        )

        # C1: Required log message within 10 seconds
        self.get_logger().info(f"TARGET BEACON: Searching for {self.colour}.")

        self.bridge = CvBridge()
        self.best_score = 0.0
        self.saved = False

        # Ensure snaps directory exists
        os.makedirs(os.path.dirname(SNAP_PATH), exist_ok=True)

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            f"Beacon search ready - looking for {self.colour} beacon."
        )

    def _get_mask(self, hsv):
        """Get colour mask for target colour."""
        lo, hi = COLOUR_RANGES.get(self.colour, COLOUR_RANGES['blue'])
        mask = cv2.inRange(
            hsv,
            np.array(lo, dtype=np.uint8),
            np.array(hi, dtype=np.uint8),
        )

        # Red wraps around the HSV hue circle - combine both ranges
        if self.colour == 'red':
            lo2, hi2 = RED_UPPER
            mask2 = cv2.inRange(
                hsv,
                np.array(lo2, dtype=np.uint8),
                np.array(hi2, dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, mask2)

        return mask

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        img_h, img_w = cv_image.shape[:2]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = self._get_mask(hsv)

        # Clean up mask with morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

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

        # Get bounding box
        bx, by, bw, bh = cv2.boundingRect(largest)

        # Calculate fractions of image dimensions
        width_frac  = bw / img_w
        height_frac = bh / img_h

        # Score rewards:
        # - Large blob area (close to beacon)
        # - Full width coverage (C4 marks)
        # - Full height coverage (C5 marks)
        score = area * width_frac * height_frac

        if score > self.best_score:
            self.best_score = score

            # Save RAW image — no filtering applied (required by marking)
            cv2.imwrite(SNAP_PATH, cv_image)

            if not self.saved:
                self.saved = True
                self.get_logger().info(
                    f"BEACON CAPTURED: {self.colour} beacon saved! "
                    f"area={area:.0f} w={width_frac:.2f} h={height_frac:.2f}"
                )
            else:
                self.get_logger().info(
                    f"BEACON UPDATED: better view saved "
                    f"(score={score:.0f} w={width_frac:.2f} h={height_frac:.2f})"
                )

    def on_shutdown(self):
        if self.saved:
            self.get_logger().info(
                f"Beacon node shutting down - {self.colour} beacon image saved."
            )
        else:
            self.get_logger().warn(
                "Beacon node shutting down - NO beacon image captured!"
            )
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
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()