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

COLOUR_RANGES = {
    'yellow': ([18,  60,  60], [38, 255, 255]),
    'green':  ([36,  30,  30], [89, 255, 255]),
    'blue':   ([85,  30,  30], [135, 255, 255]),
    'red':    ([0,  100, 100], [10, 255, 255]),
}

RED_UPPER = ([165, 100, 100], [180, 255, 255])

MIN_CONTOUR_AREA = 500
LOCK_WIDTH_FRAC  = 0.25
EDGE_MARGIN      = 10  # pixels from edge — beacon must not touch edges


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

        self.get_logger().info(f"TARGET BEACON: Searching for {self.colour}.")

        self.bridge     = CvBridge()
        self.best_score = 0.0
        self.saved      = False
        self.locked     = False

        os.makedirs(os.path.dirname(SNAP_PATH), exist_ok=True)

        self.image_sub = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            f"Beacon search ready - looking for {self.colour} beacon.")

    def _get_mask(self, hsv):
        lo, hi = COLOUR_RANGES.get(self.colour, COLOUR_RANGES['blue'])
        mask = cv2.inRange(
            hsv,
            np.array(lo, dtype=np.uint8),
            np.array(hi, dtype=np.uint8),
        )
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
        if self.locked:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        img_h, img_w = cv_image.shape[:2]

        hsv  = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = self._get_mask(hsv)

        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)

        if area < MIN_CONTOUR_AREA:
            return

        bx, by, bw, bh = cv2.boundingRect(largest)

        width_frac  = bw / img_w
        height_frac = bh / img_h

        # Beacon must NOT touch any image edge
        beacon_not_clipped = (
            bx > EDGE_MARGIN and
            by > EDGE_MARGIN and
            (bx + bw) < img_w - EDGE_MARGIN and
            (by + bh) < img_h - EDGE_MARGIN
        )

        if not beacon_not_clipped:
            return

        # Score: large area + good width + good height
        score = area * width_frac * height_frac

        if score > self.best_score:
            self.best_score = score

            cv2.imwrite(SNAP_PATH, cv_image)

            if not self.saved:
                self.saved = True
                self.get_logger().info(
                    f"BEACON CAPTURED: {self.colour} beacon saved! "
                    f"area={area:.0f} w={width_frac:.2f} h={height_frac:.2f} "
                    f"bx={bx} by={by} bw={bw} bh={bh}")
            else:
                self.get_logger().info(
                    f"BEACON UPDATED: better view "
                    f"(score={score:.0f} w={width_frac:.2f} h={height_frac:.2f})")

            # Lock when beacon fills enough of the frame
            if width_frac >= LOCK_WIDTH_FRAC:
                self.locked = True
                self.get_logger().info(
                    f"BEACON LOCKED: full image captured "
                    f"(w={width_frac:.2f}) — no more updates")

    def on_shutdown(self):
        if self.saved:
            self.get_logger().info(
                f"Beacon node shutdown - {self.colour} image saved.")
        else:
            self.get_logger().warn(
                "Beacon node shutdown - NO beacon image captured!")
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
        #while not node.shutdown:
        #    continue
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()