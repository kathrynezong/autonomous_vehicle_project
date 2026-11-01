"""Launch file for F1TENTH Autonomous Parking with Sign Detection - Team A3.

Launches:
  - front_in_parking_node (wall follow + slot detection + front-in parking + sign response + forward safety)
  - sign_detection_node (HSV + shape + temporal filtering)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        # ----- Parking Node (Tera's front-in parking with sign integration) -----
        Node(
            package='parking_perception',
            executable='front_in_parking_node',
            name='front_in_parking_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Topics
                'scan_topic': '/scan',
                'odom_topic': '/odom',
                'drive_topic': '/drive',

                # Left wall following
                'left_follow_sector_deg': 90.0,
                'left_follow_sector_width_deg': 16.0,
                'left_target_distance': 0.45,
                'left_wall_kp': 0.90,
                'left_wall_ki': 0.0,
                'left_wall_kd': 0.10,
                'max_wall_follow_steering': 0.32,
                'left_wall_max_dist': 2.0,
                'left_front_angle_deg': 50.0,
                'left_side_angle_deg': 90.0,
                'wall_follow_lookahead': 1.5,
                'steering_integral_limit': 0.4,

                # Right-side slot detection
                'right_detection_sector_deg': -90.0,
                'right_detection_sector_width_deg': 20.0,
                'near_wall_threshold': 0.45,
                'far_opening_threshold': 0.65,
                'slot_length_min': 0.50,
                'slot_length_max': 4.20,
                'edge_confirm_frames': 3,

                # Driving speeds
                'explore_speed': 0.80,
                'prepare_speed': 0.5,
                'align_speed': 0.40,
                'turn_speed': 0.30,
                'straight_speed': 0.50,

                # Parking maneuver geometry
                'turn_steering_angle': -0.28,
                'turn_angle_deg': 36.0,
                'straight_distance': 0.45,
                'align_distance_after_start': 0.15,
                'slot_position_tolerance': 0.60,
                'pre_trigger_distance': 1.00,

                # Lap detection
                'lap_min_distance': 16.0,
                'start_pose_tolerance': 0.60,

                # Forward safety (built into parking node)
                'forward_angle_deg': 10.0,
                'stop_distance_m': 0.55,
                'clear_distance_m': 0.75,
                'forward_hits_required': 3,
                'min_valid_range': 0.05,
                'max_valid_range': 8.0,
                'clear_frames_required': 3,
                'stop_hold_max_sec': 3.0,

                # Parking safety sectors
                'parking_right_sector_deg': -90.0,
                'parking_right_sector_width_deg': 16.0,
                'parking_right_min_distance': 0.22,
                'parking_front_sector_deg': 0.0,
                'parking_front_sector_width_deg': 12.0,
                'parking_front_min_distance': 0.24,

                # Sign response
                'stop_duration_sec': 2.0,
                'yield_duration_sec': 3.0,
                'playground_speed': 0.5,

                # Control loop
                'control_rate_hz': 10.0,
            }],
        ),

        # ----- Sign Detection (Jimmy's v5 HSV + temporal filter) -----
        Node(
            package='parking_perception',
            executable='sign_detection_node',
            name='sign_detection_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'min_sign_area': 1500,
                'max_sign_area': 150000,
                'sign_cooldown_sec': 5.0,
                'startup_delay_sec': 3.0,
                'window_size': 8,
                'min_hits': 5,
            }],
        ),

        # LiDAR safety is built into front_in_parking_node (forward_angle_deg, stop_distance_m, etc.)
        # No separate safety node needed — Tera's implementation auto-disables during parking.
    ])
