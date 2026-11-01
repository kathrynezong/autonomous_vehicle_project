#!/usr/bin/env python3
"""Sign detection node using HSV, shape, and temporal filtering.

Uses simple HSV color segmentation and contour shape analysis per frame,
then requires N detections within a sliding window of W frames before
publishing a confirmed detection. This eliminates transient false
positives while reliably detecting real signs.

Supported signs:
  STOP               - Red octagon        -> stop for 2 s
  YIELD              - Red inverted triangle -> 50% speed for 3 s
  PLAYGROUND_ZONE    - Yellow diamond      -> limit to 0.5 m/s
  END_PLAYGROUND_ZONE - Yellow rectangle   -> resume normal speed
"""

from __future__ import annotations

import time
from collections import defaultdict

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from parking_perception.perception.sign_detector import (
    SIGN_COLORS,
    SIGN_TYPES,
    SignDetector,
)


class SignDetectionNode(Node):

    def __init__(self):
        super().__init__('sign_detection_node')

        self.declare_parameter('min_sign_area', 1500)
        self.declare_parameter('max_sign_area', 150000)
        self.declare_parameter('sign_cooldown_sec', 5.0)
        self.declare_parameter('startup_delay_sec', 3.0)
        self.declare_parameter('window_size', 8)
        self.declare_parameter('min_hits', 5)

        self.min_area = self.get_parameter('min_sign_area').value
        self.max_area = self.get_parameter('max_sign_area').value
        self.cooldown_sec = self.get_parameter('sign_cooldown_sec').value
        self.startup_delay = self.get_parameter('startup_delay_sec').value
        self.window_size = self.get_parameter('window_size').value
        self.min_hits = self.get_parameter('min_hits').value
        self.detector = SignDetector(self.min_area, self.max_area)

        self.sign_pub = self.create_publisher(String, '/parking/sign_detected', 10)
        self.debug_pub = self.create_publisher(Image, '/parking/sign_debug', 10)
        self.create_subscription(
            Image, '/camera/color/image_raw', self.camera_callback, 10)

        self.bridge = CvBridge()
        self._start_time = time.time()

        # Temporal filter state
        self._history = defaultdict(list)
        self._last_confirmed = {}

        self.get_logger().info(
            f'SignDetectionNode v5 started. min_area={self.min_area}, '
            f'window={self.window_size}, min_hits={self.min_hits}, '
            f'cooldown={self.cooldown_sec}s')

    # ==================================================================
    # Temporal filter
    # ==================================================================

    def _temporal_update(self, raw_types):
        """Sliding window: confirm if detected in min_hits of last window_size frames."""
        detected_set = set(raw_types)
        confirmed = []

        for sign_type in SIGN_TYPES:
            self._history[sign_type].append(sign_type in detected_set)
            if len(self._history[sign_type]) > self.window_size:
                self._history[sign_type].pop(0)

            hits = sum(self._history[sign_type])
            if hits >= self.min_hits:
                last = self._last_confirmed.get(sign_type, 0)
                if time.time() - last > self.cooldown_sec:
                    confirmed.append(sign_type)
                    self._last_confirmed[sign_type] = time.time()
                    self._history[sign_type] = []

        return confirmed

    # ==================================================================
    # Main callback
    # ==================================================================

    def camera_callback(self, msg: Image):
        if time.time() - self._start_time < self.startup_delay:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Only detect in bottom half of frame
        h, w = frame.shape[:2]
        roi = frame[int(h * 0.5):h, :, :]

        # Per-frame raw detection
        raw_roi = self.detector.detect(roi)
        # Offset bboxes back to full frame coords
        y_off = int(h * 0.5)
        raw = [(s, (x, y + y_off, bw, bh)) for s, (x, y, bw, bh) in raw_roi]
        raw_types = [r[0] for r in raw]

        # Temporal filtering
        confirmed = self._temporal_update(raw_types)

        # Publish confirmed (or NONE)
        sign_msg = String()
        if confirmed:
            sign_msg.data = confirmed[0]  # highest priority confirmed
            self.get_logger().info(f'CONFIRMED: {confirmed[0]}')
        else:
            sign_msg.data = 'NONE'
        self.sign_pub.publish(sign_msg)

        # Debug image
        debug = frame.copy()
        for sign_type, (x, y, w, h) in raw:
            color = SIGN_COLORS.get(sign_type, (255, 255, 255))
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            cv2.putText(debug, sign_type, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SignDetectionNode stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
