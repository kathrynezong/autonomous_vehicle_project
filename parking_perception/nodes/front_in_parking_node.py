import math
import time
from typing import Optional, Tuple

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from parking_perception.control.parking_state import (
    ParkingMode,
    RightEdgePhase,
    SlotEstimate,
    normalize_angle,
    yaw_from_quaternion,
)


class FrontInParkingNode(Node):
    """Map a right-side parking slot on lap 1 and park into it on lap 2."""

    def __init__(self) -> None:
        super().__init__('front_in_parking_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')

        self.declare_parameter('left_follow_sector_deg', 90.0)
        self.declare_parameter('left_follow_sector_width_deg', 16.0)
        self.declare_parameter('left_target_distance', 0.45)
        self.declare_parameter('left_wall_kp', 0.90)
        self.declare_parameter('left_wall_ki', 0.0)
        self.declare_parameter('left_wall_kd', 0.10)
        self.declare_parameter('max_wall_follow_steering', 0.32)
        self.declare_parameter('left_wall_max_dist', 2.0)
        self.declare_parameter('left_front_angle_deg', 50.0)
        self.declare_parameter('left_side_angle_deg', 90.0)
        self.declare_parameter('wall_follow_lookahead', 1.5)
        self.declare_parameter('steering_integral_limit', 0.4)

        self.declare_parameter('right_detection_sector_deg', -90.0)
        self.declare_parameter('right_detection_sector_width_deg', 20.0)
        self.declare_parameter('near_wall_threshold', 0.45)
        self.declare_parameter('far_opening_threshold', 0.65)
        self.declare_parameter('slot_length_min', 0.50)
        self.declare_parameter('slot_length_max', 4.20)
        self.declare_parameter('edge_confirm_frames', 3)

        self.declare_parameter('explore_speed', 0.80)
        self.declare_parameter('prepare_speed', 0.55)
        self.declare_parameter('align_speed', 0.40)
        self.declare_parameter('turn_speed', 0.30)
        self.declare_parameter('turn_steering_angle', -0.28)
        self.declare_parameter('turn_angle_deg', 36.0)
        self.declare_parameter('straight_speed', 0.60)
        self.declare_parameter('straight_distance', 0.45)
        self.declare_parameter('control_rate_hz', 10.0)

        self.declare_parameter('pre_trigger_distance', 1.00)
        self.declare_parameter('align_distance_after_start', 0.15)
        self.declare_parameter('slot_position_tolerance', 0.60)
        self.declare_parameter('lap_min_distance', 16.0)
        self.declare_parameter('start_pose_tolerance', 0.60)

        self.declare_parameter('forward_angle_deg', 20.0)
        self.declare_parameter('stop_distance_m', 0.65)
        self.declare_parameter('clear_distance_m', 0.80)
        self.declare_parameter('forward_hits_required', 3)
        self.declare_parameter('min_valid_range', 0.05)
        self.declare_parameter('max_valid_range', 8.0)
        self.declare_parameter('clear_frames_required', 3)
        self.declare_parameter('stop_hold_max_sec', 3.0)
        self.declare_parameter('parking_right_sector_deg', -90.0)
        self.declare_parameter('parking_right_sector_width_deg', 16.0)
        self.declare_parameter('parking_right_min_distance', 0.22)
        self.declare_parameter('parking_front_sector_deg', 0.0)
        self.declare_parameter('parking_front_sector_width_deg', 12.0)
        self.declare_parameter('parking_front_min_distance', 0.24)

        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        self.left_follow_sector_center = math.radians(
            float(self.get_parameter('left_follow_sector_deg').value)
        )
        self.left_follow_sector_width = math.radians(
            float(self.get_parameter('left_follow_sector_width_deg').value)
        )
        self.left_target_distance = float(
            self.get_parameter('left_target_distance').value
        )
        self.left_wall_kp = float(self.get_parameter('left_wall_kp').value)
        self.left_wall_ki = float(self.get_parameter('left_wall_ki').value)
        self.left_wall_kd = float(self.get_parameter('left_wall_kd').value)
        self.max_wall_follow_steering = float(
            self.get_parameter('max_wall_follow_steering').value
        )
        self.left_wall_max_dist = float(
            self.get_parameter('left_wall_max_dist').value
        )
        self.left_front_angle_deg = float(
            self.get_parameter('left_front_angle_deg').value
        )
        self.left_side_angle_deg = float(
            self.get_parameter('left_side_angle_deg').value
        )
        self.wall_follow_lookahead = float(
            self.get_parameter('wall_follow_lookahead').value
        )
        self.steering_integral_limit = float(
            self.get_parameter('steering_integral_limit').value
        )

        self.right_detection_sector_center = math.radians(
            float(self.get_parameter('right_detection_sector_deg').value)
        )
        self.right_detection_sector_width = math.radians(
            float(self.get_parameter('right_detection_sector_width_deg').value)
        )
        self.near_wall_threshold = float(
            self.get_parameter('near_wall_threshold').value
        )
        self.far_opening_threshold = float(
            self.get_parameter('far_opening_threshold').value
        )
        self.slot_length_min = float(self.get_parameter('slot_length_min').value)
        self.slot_length_max = float(self.get_parameter('slot_length_max').value)
        self.edge_confirm_frames = int(
            self.get_parameter('edge_confirm_frames').value
        )

        self.explore_speed = float(self.get_parameter('explore_speed').value)
        self.prepare_speed = float(self.get_parameter('prepare_speed').value)
        self.align_speed = float(self.get_parameter('align_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_steering_angle = float(
            self.get_parameter('turn_steering_angle').value
        )
        self.turn_angle = math.radians(
            float(self.get_parameter('turn_angle_deg').value)
        )
        self.straight_speed = float(self.get_parameter('straight_speed').value)
        self.straight_distance = float(self.get_parameter('straight_distance').value)

        self.pre_trigger_distance = float(
            self.get_parameter('pre_trigger_distance').value
        )
        self.align_distance_after_start = float(
            self.get_parameter('align_distance_after_start').value
        )
        self.slot_position_tolerance = float(
            self.get_parameter('slot_position_tolerance').value
        )
        self.lap_min_distance = float(self.get_parameter('lap_min_distance').value)
        self.start_pose_tolerance = float(
            self.get_parameter('start_pose_tolerance').value
        )

        self.forward_angle = math.radians(
            float(self.get_parameter('forward_angle_deg').value)
        )
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.clear_distance_m = float(self.get_parameter('clear_distance_m').value)
        self.forward_hits_required = int(
            self.get_parameter('forward_hits_required').value
        )
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.max_valid_range = float(self.get_parameter('max_valid_range').value)
        self.clear_frames_required = int(
            self.get_parameter('clear_frames_required').value
        )
        self.stop_hold_max_sec = float(self.get_parameter('stop_hold_max_sec').value)
        self.parking_right_sector_center = math.radians(
            float(self.get_parameter('parking_right_sector_deg').value)
        )
        self.parking_right_sector_width = math.radians(
            float(self.get_parameter('parking_right_sector_width_deg').value)
        )
        self.parking_right_min_distance = float(
            self.get_parameter('parking_right_min_distance').value
        )
        self.parking_front_sector_center = math.radians(
            float(self.get_parameter('parking_front_sector_deg').value)
        )
        self.parking_front_sector_width = math.radians(
            float(self.get_parameter('parking_front_sector_width_deg').value)
        )
        self.parking_front_min_distance = float(
            self.get_parameter('parking_front_min_distance').value
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(String, '/parking/sign_detected', self.sign_callback, 10)

        # Sign response parameters
        self.declare_parameter('stop_duration_sec', 2.0)
        self.declare_parameter('yield_duration_sec', 3.0)
        self.declare_parameter('playground_speed', 0.5)
        self.stop_duration = float(self.get_parameter('stop_duration_sec').value)
        self.yield_duration = float(self.get_parameter('yield_duration_sec').value)
        self.playground_speed = float(self.get_parameter('playground_speed').value)

        # Sign state
        self._sign_action = None       # 'STOP', 'YIELD', 'PLAYGROUND_ZONE', etc.
        self._sign_action_start = 0.0
        self._playground_active = False
        self._original_explore_speed = None

        control_period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(control_period, self.control_loop)

        self.mode = ParkingMode.EXPLORE
        self.latest_scan: Optional[LaserScan] = None
        self.current_distance = 0.0
        self.current_yaw = 0.0
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None
        self.lap_count = 0
        self.ready_for_new_lap = False
        self.lap_start_distance = 0.0
        self.current_lap_distance = 0.0
        self.first_lap_length: Optional[float] = None

        self.edge_phase = RightEdgePhase.SEEKING_NEAR
        self.wall_frames = 0
        self.edge_frames = 0
        self.opening_start_s: Optional[float] = None
        self._opening_first_far_s: Optional[float] = None  # Bug2: first far frame position
        self._right_wall_logged = False

        self.mapped_slot: Optional[SlotEstimate] = None
        self.first_slot_detected_distance: Optional[float] = None
        self.confirmed_slot_start_s: Optional[float] = None
        self.prepare_logged = False
        self._prepare_passed_end_warned = False

        self.stop_active = False
        self.clear_count = 0
        self._stop_active_start: Optional[float] = None  # monotonic time when stop_active was set
        self._wall_follow_integral = 0.0
        self._prev_wall_follow_error = 0.0
        self._prev_wall_follow_time = None

        self._second_pass_open_frames = 0

        self.turn_start_yaw: Optional[float] = None
        self.straight_start_s: Optional[float] = None

        self.get_logger().info('Front-in parking node started.')
        self.get_logger().info('Strategy: left wall-follow, map slot on lap 1, park on lap 2.')

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quaternion(orientation.z, orientation.w)

        if self.prev_x is not None and self.prev_y is not None:
            self.current_distance += math.hypot(
                position.x - self.prev_x,
                position.y - self.prev_y,
            )
        else:
            self.start_x = position.x
            self.start_y = position.y
            self.start_yaw = self.current_yaw

        self.prev_x = position.x
        self.prev_y = position.y

        self.current_lap_distance = self.current_distance

    def update_lap_counter(self, x: float, y: float) -> None:
        """Lap counting is intentionally disabled for continuous slot detection."""
        return

    def sign_callback(self, msg: String) -> None:
        """Handle confirmed sign detections from sign_detection_node."""
        sign = msg.data.strip().upper()
        if not sign or sign == 'NONE':
            return
        # Only respond during EXPLORE or PREPARE_TO_PARK
        if self.mode not in (ParkingMode.EXPLORE, ParkingMode.PREPARE_TO_PARK):
            return

        if sign == 'STOP':
            self._sign_action = 'STOP'
            self._sign_action_start = time.monotonic()
            self.get_logger().info('SIGN: STOP — stopping for 2s')
        elif sign == 'YIELD':
            self._sign_action = 'YIELD'
            self._sign_action_start = time.monotonic()
            self.get_logger().info('SIGN: YIELD — slowing down for 3s')
        elif sign == 'PLAYGROUND_ZONE':
            self._playground_active = True
            if self._original_explore_speed is None:
                self._original_explore_speed = self.explore_speed
            self.explore_speed = self.playground_speed
            self.get_logger().info(f'SIGN: PLAYGROUND_ZONE — speed={self.playground_speed}')
        elif sign == 'END_PLAYGROUND_ZONE':
            self._playground_active = False
            if self._original_explore_speed is not None:
                self.explore_speed = self._original_explore_speed
                self._original_explore_speed = None
            self.get_logger().info('SIGN: END_PLAYGROUND_ZONE — speed restored')

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.update_forward_safety(msg)

    def control_loop(self) -> None:
        if self.latest_scan is None or self.prev_x is None:
            return

        if self.stop_active:
            now = time.monotonic()
            if self._stop_active_start is None:
                self._stop_active_start = now
            elif (now - self._stop_active_start) >= self.stop_hold_max_sec:
                # Obstacle has not cleared within the timeout window.  Permanent walls
                # (e.g. parking-bay walls entering the forward cone during EXPLORE) will
                # never self-clear, so release the hold and resume rather than freezing.
                self.stop_active = False
                self.clear_count = 0
                self._stop_active_start = None
                self.get_logger().warn(
                    f'Forward safety hold exceeded {self.stop_hold_max_sec:.1f} s. '
                    'Force-releasing and resuming.'
                )
            else:
                self.publish_drive(0.0, 0.0)
                return

        # Handle active sign actions (STOP / YIELD)
        if self._sign_action == 'STOP':
            elapsed = time.monotonic() - self._sign_action_start
            if elapsed < self.stop_duration:
                self.publish_drive(0.0, 0.0)
                return
            self._sign_action = None
        elif self._sign_action == 'YIELD':
            elapsed = time.monotonic() - self._sign_action_start
            if elapsed >= self.yield_duration:
                self._sign_action = None
            # Don't return — let the car drive but at reduced speed
            # (handled by _apply_yield_speed below)

        if self.mode == ParkingMode.EXPLORE:
            self.handle_explore()
        elif self.mode == ParkingMode.PREPARE_TO_PARK:
            self.handle_prepare_to_park()
        elif self.mode == ParkingMode.ALIGN_TO_SLOT:
            self.handle_align_to_slot()
        elif self.mode == ParkingMode.TURN_IN:
            self.handle_turn_in()
        elif self.mode == ParkingMode.STRAIGHTEN:
            self.handle_straighten()
        elif self.mode == ParkingMode.DONE:
            self.publish_drive(0.0, 0.0)

    def handle_explore(self) -> None:
        if self.mapped_slot is None:
            # First pass: run full slot detector until exactly one slot is found.
            slot_estimate = self.update_right_slot_detector()
            if slot_estimate is not None:
                self.mapped_slot = slot_estimate
                self.first_slot_detected_distance = self.current_distance
                self.get_logger().info(
                    'FIRST SLOT MAPPED: '
                    f'start={slot_estimate.start_s:.2f} m, '
                    f'end={slot_estimate.end_s:.2f} m, '
                    f'length={slot_estimate.length:.2f} m.'
                )
        else:
            # Second pass: slot detector is frozen. After one lap, wait for the
            # right side to open up again and enter parking mode immediately.
            if (
                self.first_slot_detected_distance is not None
                and (self.current_distance - self.first_slot_detected_distance) >= self.lap_min_distance
            ):
                side_distance = self.get_sector_average_range(
                    self.latest_scan,
                    self.right_detection_sector_center,
                    self.right_detection_sector_width,
                )
                if side_distance is not None and side_distance <= self.near_wall_threshold:
                    self._second_pass_open_frames += 1
                    if self._second_pass_open_frames >= self.edge_confirm_frames:
                        self.confirmed_slot_start_s = self.current_distance
                        self.stop_active = False
                        self.clear_count = 0
                        self._stop_active_start = None
                        self.mode = ParkingMode.ALIGN_TO_SLOT
                        self.get_logger().info(
                            f'Second pass: right wall detected at {self.current_distance:.2f} m. '
                            'Entering parking mode.'
                        )
                        self.publish_drive(0.0, 0.0)
                        return
                else:
                    self._second_pass_open_frames = 0

        self.publish_drive(
            self.explore_speed,
            self.compute_left_wall_follow_steering(self.latest_scan),
        )

    def handle_prepare_to_park(self) -> None:
        if not self.prepare_logged:
            self.prepare_logged = True
            self.get_logger().info('Approaching recorded slot position and waiting for local confirmation.')

        # Fix4: distance-based timeout — if we passed the expected slot end without confirmation,
        # go back to EXPLORE and retry on the next lap (_prepare_attempted_this_lap stays True
        # so we won't re-enter PREPARE again until update_lap_counter resets it)
        if self.mapped_slot is not None and not self._prepare_passed_end_warned:
            overrun_s = self.mapped_slot.end_s + self.slot_position_tolerance
            if self.current_lap_distance > overrun_s:
                self._prepare_passed_end_warned = True
                self.get_logger().warn(
                    f'Passed expected slot end ({overrun_s:.2f} m) without confirmation. '
                    'Resuming exploration; will retry next lap.'
                )
                self.reset_right_slot_detector('Prepare timeout: resetting detector.')
                self.mode = ParkingMode.EXPLORE
                self.publish_drive(0.0, 0.0)
                return

        slot_estimate = self.update_right_slot_detector()
        if slot_estimate is not None and self.mapped_slot is not None and self.first_slot_detected_distance is not None:
            relative_new = slot_estimate.start_s - self.current_distance
            relative_mapped = self.mapped_slot.start_s - self.first_slot_detected_distance
            start_error = abs(relative_new - relative_mapped)
            if start_error <= self.slot_position_tolerance:
                self.confirmed_slot_start_s = slot_estimate.start_s
                # Fix2: clear any stop_active that fired during PREPARE_TO_PARK approach;
                # forward safety is now bypassed for the rest of the parking sequence
                self.stop_active = False
                self.clear_count = 0
                self._stop_active_start = None
                self.mode = ParkingMode.ALIGN_TO_SLOT
                self.get_logger().info(
                    'Recorded slot confirmed on lap 2. '
                    f'start error={start_error:.2f} m.'
                )
                self.publish_drive(0.0, 0.0)
                return
            self.get_logger().info(
                f'Ignored slot candidate due to position mismatch: {start_error:.2f} m.'
            )

        self.publish_drive(
            self.prepare_speed,
            self.compute_left_wall_follow_steering(self.latest_scan),
        )

    def handle_align_to_slot(self) -> None:
        if self.confirmed_slot_start_s is None:
            self.mode = ParkingMode.PREPARE_TO_PARK
            return

        traveled_since_confirm = self.current_lap_distance - self.confirmed_slot_start_s
        if traveled_since_confirm >= self.align_distance_after_start:
            self.mode = ParkingMode.TURN_IN
            self.turn_start_yaw = self.current_yaw
            self.get_logger().info('Reached parking entry trigger. Starting front-in turn.')
            self.publish_drive(0.0, 0.0)
            return

        self.publish_drive(
            self.align_speed,
            self.compute_left_wall_follow_steering(self.latest_scan),
        )

    def handle_turn_in(self) -> None:
        if self.turn_start_yaw is None:
            self.turn_start_yaw = self.current_yaw

        right_clearance = self.get_sector_min_range(
            self.latest_scan,
            self.parking_right_sector_center,
            self.parking_right_sector_width,
        )
        if (
            right_clearance is not None
            and right_clearance <= self.parking_right_min_distance
        ):
            self.mode = ParkingMode.STRAIGHTEN
            self.straight_start_s = self.current_distance
            self.get_logger().warn(
                f'Right wall too close during turn ({right_clearance:.2f} m). '
                'Stopping turn early and straightening.'
            )
            self.publish_drive(0.0, 0.0)
            return

        yaw_delta = normalize_angle(self.current_yaw - self.turn_start_yaw)
        if abs(yaw_delta) >= abs(self.turn_angle):
            self.mode = ParkingMode.STRAIGHTEN
            self.straight_start_s = self.current_distance
            self.get_logger().info('Turn complete. Driving straight into the parking space.')
            self.publish_drive(0.0, 0.0)
            return

        self.publish_drive(self.turn_speed, self.turn_steering_angle)

    def handle_straighten(self) -> None:
        if self.straight_start_s is None:
            self.straight_start_s = self.current_distance

        front_clearance = self.get_sector_min_range(
            self.latest_scan,
            self.parking_front_sector_center,
            self.parking_front_sector_width,
        )
        if (
            front_clearance is not None
            and front_clearance <= self.parking_front_min_distance
        ):
            self.mode = ParkingMode.DONE
            self.get_logger().warn(
                f'Front wall too close while parking ({front_clearance:.2f} m). '
                'Stopping parking maneuver.'
            )
            self.publish_drive(0.0, 0.0)
            return

        if (self.current_distance - self.straight_start_s) >= self.straight_distance:
            self.mode = ParkingMode.DONE
            self.get_logger().info('Front-in parking finished.')
            self.publish_drive(0.0, 0.0)
            return

        self.publish_drive(self.straight_speed, 0.0)

    def update_right_slot_detector(self) -> Optional[SlotEstimate]:
        side_distance = self.get_sector_average_range(
            self.latest_scan,
            self.right_detection_sector_center,
            self.right_detection_sector_width,
        )
        if side_distance is None:
            return None

        sees_near = side_distance <= self.near_wall_threshold
        sees_far = side_distance >= self.far_opening_threshold
        if self.edge_phase == RightEdgePhase.SEEKING_NEAR:
            if sees_near:
                self.wall_frames += 1
                if self.wall_frames >= self.edge_confirm_frames:
                    self.edge_phase = RightEdgePhase.SEEKING_FIRST_FAR
                    if not self._right_wall_logged:
                        self.get_logger().info(
                            f'RIGHT WALL FOUND: right_avg={side_distance:.2f} m'
                        )
                        self._right_wall_logged = True
            else:
                self.wall_frames = 0
            return None

        if self.edge_phase == RightEdgePhase.SEEKING_FIRST_FAR:
            if sees_far:
                if self.edge_frames == 0:
                    self._opening_first_far_s = self.current_lap_distance  # Bug2: record first frame
                self.edge_frames += 1
                if self.edge_frames >= self.edge_confirm_frames:
                    self.edge_phase = RightEdgePhase.MEASURING_OPEN
                    self.opening_start_s = self._opening_first_far_s  # Bug2: use first frame position
                    self.edge_frames = 0
                    self.get_logger().info(
                        'OPENING START: '
                        f'lap_s={self.opening_start_s:.2f} m, '
                        f'right_avg={side_distance:.2f} m'
                    )
            elif sees_near:
                self.edge_frames = 0
                self._opening_first_far_s = None
            else:
                self.edge_frames = 0
                self._opening_first_far_s = None
            return None

        if self.edge_phase == RightEdgePhase.MEASURING_OPEN:
            if self.opening_start_s is None:
                self.opening_start_s = self.current_lap_distance

            opening_length = self.current_lap_distance - self.opening_start_s
            if opening_length > self.slot_length_max:
                self.reset_right_slot_detector(
                    f'Reset oversized opening: {opening_length:.2f} m.'
                )
                return None

            if sees_near:
                self.edge_frames += 1
                if self.edge_frames >= self.edge_confirm_frames:
                    slot_end_s = self.current_lap_distance
                    slot_length = slot_end_s - self.opening_start_s
                    self.get_logger().info(
                        'OPENING END: '
                        f'lap_s={slot_end_s:.2f} m, '
                        f'length={slot_length:.2f} m, '
                        f'right_avg={side_distance:.2f} m'
                    )
                    if self.slot_length_min <= slot_length <= self.slot_length_max:
                        slot = SlotEstimate(
                            start_s=self.opening_start_s,
                            end_s=slot_end_s,
                            center_s=(self.opening_start_s + slot_end_s) / 2.0,
                            length=slot_length,
                        )
                        self.reset_right_slot_detector(
                            f'SLOT ACCEPTED: start={slot.start_s:.2f} m, '
                            f'end={slot.end_s:.2f} m, length={slot.length:.2f} m'
                        )
                        return slot
                    self.reset_right_slot_detector(
                        f'SLOT REJECTED: length={slot_length:.2f} m'
                    )
                    return None
            else:
                self.edge_frames = 0

        return None

    def reset_right_slot_detector(self, reason: str) -> None:
        self.get_logger().info(reason)
        self.edge_phase = RightEdgePhase.SEEKING_NEAR
        self.wall_frames = 0
        self.edge_frames = 0
        self.opening_start_s = None
        self._opening_first_far_s = None
        self._right_wall_logged = False

    def update_forward_safety(self, scan: LaserScan) -> None:
        # Disable forward safety once the car is in the parking sequence.
        # PREPARE_TO_PARK included: the parking wall can enter the forward cone during the
        # approach and set stop_active; because stop_active short-circuits the entire control
        # loop, the car would freeze and the timeout's distance check would never advance,
        # creating a permanent deadlock with no self-recovery path.
        if self.mode in (
            ParkingMode.PREPARE_TO_PARK,
            ParkingMode.ALIGN_TO_SLOT,
            ParkingMode.TURN_IN,
            ParkingMode.STRAIGHTEN,
            ParkingMode.DONE,
        ):
            return

        front_range, close_hits = self.get_front_obstacle_stats(scan)

        if front_range is None:
            self.clear_count += 1
            if self.clear_count >= self.clear_frames_required and self.stop_active:
                self.stop_active = False
                self._stop_active_start = None
                self.get_logger().info('No valid front lidar returns. Releasing stop hold.')
            return

        if close_hits >= self.forward_hits_required:
            self.clear_count = 0
            if not self.stop_active:
                self.get_logger().warn(
                    f'Obstacle detected at {front_range:.2f} m with {close_hits} close hits. Holding position.'
                )
                self._stop_active_start = None  # reset timer so control_loop starts counting fresh
            self.stop_active = True
            return

        if front_range >= self.clear_distance_m:
            self.clear_count += 1
            if self.clear_count >= self.clear_frames_required and self.stop_active:
                self.stop_active = False
                self._stop_active_start = None
                self.get_logger().info(
                    f'Front sector clear again at {front_range:.2f} m. Resuming motion.'
                )
            return

        self.clear_count = 0

    def compute_left_wall_follow_steering(self, scan: LaserScan) -> float:
        dist_a = self.get_range_at_angle(scan, self.left_front_angle_deg)
        dist_b = self.get_range_at_angle(scan, self.left_side_angle_deg)

        if dist_a is None or dist_b is None:
            return 0.15

        if dist_b > self.left_wall_max_dist:
            front_dist = self.get_range_at_angle(scan, 0.0)
            if front_dist is not None and front_dist < 0.8:
                return 0.4
            return 0.15

        theta = math.radians(abs(self.left_side_angle_deg - self.left_front_angle_deg))
        alpha = math.atan2(
            dist_a * math.cos(theta) - dist_b,
            dist_a * math.sin(theta)
        )
        current_dist = dist_b * math.cos(alpha)
        projected_dist = current_dist + self.wall_follow_lookahead * math.sin(alpha)
        error = self.left_target_distance - projected_dist

        now = self.get_clock().now()
        if self._prev_wall_follow_time is None:
            dt = 0.025
        else:
            dt = (now - self._prev_wall_follow_time).nanoseconds / 1e9
            if dt <= 1e-4:
                dt = 1e-4
        self._prev_wall_follow_time = now

        self._wall_follow_integral += error * dt
        self._wall_follow_integral = max(
            -self.steering_integral_limit,
            min(self.steering_integral_limit, self._wall_follow_integral),
        )

        derivative = (error - self._prev_wall_follow_error) / dt
        self._prev_wall_follow_error = error

        steering_cmd = (
            self.left_wall_kp * error
            + self.left_wall_ki * self._wall_follow_integral
            + self.left_wall_kd * derivative
        )
        steering_angle = max(
            -self.max_wall_follow_steering,
            min(self.max_wall_follow_steering, steering_cmd),
        )

        return -steering_angle

    def get_range_at_angle(self, scan: LaserScan, angle_deg: float) -> Optional[float]:
        angle_rad = math.radians(angle_deg)
        center_idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
        window = 3
        start_idx = max(0, center_idx - window)
        end_idx = min(len(scan.ranges), center_idx + window + 1)

        values = []
        for distance in scan.ranges[start_idx:end_idx]:
            if not math.isfinite(distance):
                continue
            if distance < self.min_valid_range or distance > self.max_valid_range:
                continue
            values.append(distance)

        if not values:
            return None

        return sum(values) / len(values)

    def get_sector_average_range(
        self,
        scan: LaserScan,
        sector_center: float,
        sector_width: float,
    ) -> Optional[float]:
        min_angle = sector_center - (sector_width / 2.0)
        max_angle = sector_center + (sector_width / 2.0)

        values = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if angle < min_angle or angle > max_angle:
                continue
            if not math.isfinite(distance):
                continue
            if distance < self.min_valid_range or distance > self.max_valid_range:
                continue
            values.append(distance)

        if not values:
            return None

        return sum(values) / len(values)

    def get_sector_min_range(
        self,
        scan: LaserScan,
        sector_center: float,
        sector_width: float,
    ) -> Optional[float]:
        min_angle = sector_center - (sector_width / 2.0)
        max_angle = sector_center + (sector_width / 2.0)

        min_range: Optional[float] = None
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if angle < min_angle or angle > max_angle:
                continue
            if not math.isfinite(distance):
                continue
            if distance < self.min_valid_range or distance > self.max_valid_range:
                continue
            if min_range is None or distance < min_range:
                min_range = distance

        return min_range

    def get_front_min_range(self, scan: LaserScan) -> Optional[float]:
        front_range, _ = self.get_front_obstacle_stats(scan)
        return front_range

    def get_front_obstacle_stats(self, scan: LaserScan) -> Tuple[Optional[float], int]:
        min_range: Optional[float] = None
        close_hits = 0
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > self.forward_angle:
                continue
            if not math.isfinite(distance):
                continue
            if distance < self.min_valid_range or distance > self.max_valid_range:
                continue
            if min_range is None or distance < min_range:
                min_range = distance
            if distance <= self.stop_distance_m:
                close_hits += 1

        return min_range, close_hits

    def publish_drive(self, speed: float, steering_angle: float) -> None:
        # Apply YIELD speed reduction (50% of current speed)
        if self._sign_action == 'YIELD' and speed > 0:
            speed = speed * 0.5
        msg = AckermannDriveStamped()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontInParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
