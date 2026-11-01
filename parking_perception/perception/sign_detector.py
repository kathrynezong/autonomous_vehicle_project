"""OpenCV traffic-sign detection independent of ROS transport."""

from typing import Optional

import cv2
import numpy as np


SIGN_TYPES = ('STOP', 'YIELD', 'PLAYGROUND_ZONE', 'END_PLAYGROUND_ZONE')

SIGN_COLORS = {
    'STOP': (0, 0, 255),
    'YIELD': (0, 128, 255),
    'PLAYGROUND_ZONE': (0, 255, 255),
    'END_PLAYGROUND_ZONE': (0, 200, 200),
}


class SignDetector:
    """Detect supported signs in a BGR camera frame."""

    RED_LOW1 = np.array([0, 80, 80])
    RED_HIGH1 = np.array([10, 255, 255])
    RED_LOW2 = np.array([160, 80, 80])
    RED_HIGH2 = np.array([180, 255, 255])
    YELLOW_LOW = np.array([15, 80, 100])
    YELLOW_HIGH = np.array([40, 255, 255])

    def __init__(self, min_area: float, max_area: float) -> None:
        self.min_area = min_area
        self.max_area = max_area

    def detect(self, frame):
        """Return a list of ``(sign_type, bounding_box)`` detections."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.RED_LOW1, self.RED_HIGH1),
            cv2.inRange(hsv, self.RED_LOW2, self.RED_HIGH2),
        )
        detections.extend(self._detections_from_mask(red_mask, self._classify_red))

        yellow_mask = cv2.inRange(hsv, self.YELLOW_LOW, self.YELLOW_HIGH)
        detections.extend(
            self._detections_from_mask(yellow_mask, self._classify_yellow)
        )
        return detections

    def _detections_from_mask(self, mask, classifier):
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []
        for contour in contours:
            if not self._passes_filter(contour):
                continue
            sign_type = classifier(contour)
            if sign_type:
                detections.append((sign_type, cv2.boundingRect(contour)))
        return detections

    def _passes_filter(self, contour) -> bool:
        area = cv2.contourArea(contour)
        if area < self.min_area or area > self.max_area:
            return False
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        return hull_area >= 1 and area / hull_area >= 0.5

    @staticmethod
    def _classify_red(contour) -> Optional[str]:
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        vertex_count = len(polygon)
        _, _, width, height = cv2.boundingRect(contour)
        aspect = width / height if height > 0 else 0

        if 7 <= vertex_count <= 10 and 0.7 < aspect < 1.4:
            return 'STOP'

        if 3 <= vertex_count <= 4:
            points = polygon.reshape(-1, 2)
            sorted_by_y = points[points[:, 1].argsort()]
            if len(sorted_by_y) >= 3:
                top_two = sorted_by_y[:2]
                top_width = abs(int(top_two[0][0]) - int(top_two[1][0]))
                total_height = sorted_by_y[-1][1] - sorted_by_y[0][1]
                if total_height > 20 and top_width > total_height * 0.5:
                    return 'YIELD'
        return None

    @staticmethod
    def _classify_yellow(contour) -> Optional[str]:
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        if len(polygon) != 4:
            return None

        _, _, width, height = cv2.boundingRect(contour)
        aspect = width / height if height > 0 else 0
        fill_ratio = cv2.contourArea(contour) / max(width * height, 1)

        if 0.7 < aspect < 1.4 and fill_ratio < 0.72:
            return 'PLAYGROUND_ZONE'
        if height > width * 1.1 and fill_ratio > 0.75:
            return 'END_PLAYGROUND_ZONE'
        return None
