"""Shared state models and geometry helpers for parking control."""

import math
from dataclasses import dataclass
from enum import Enum


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(z: float, w: float) -> float:
    """Return planar yaw from a quaternion that only rotates around z."""
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


@dataclass
class SlotEstimate:
    """Parking-space measurements recorded during exploration."""

    start_s: float
    end_s: float
    center_s: float
    length: float


class ParkingMode(Enum):
    """High-level states in the front-in parking maneuver."""

    EXPLORE = 'explore'
    PREPARE_TO_PARK = 'prepare_to_park'
    ALIGN_TO_SLOT = 'align_to_slot'
    TURN_IN = 'turn_in'
    STRAIGHTEN = 'straighten'
    DONE = 'done'


class RightEdgePhase(Enum):
    """Phases used to detect a parking opening from right-side LiDAR."""

    SEEKING_NEAR = 'seeking_near'
    SEEKING_FIRST_FAR = 'seeking_first_far'
    MEASURING_OPEN = 'measuring_open'
