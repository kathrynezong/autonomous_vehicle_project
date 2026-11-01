# Autonomous Parking with Traffic Sign Detection

A ROS 2 perception and control system that enables an F1TENTH vehicle to navigate a closed track, recognize traffic signs, detect an open parking space, and perform a front-in parking maneuver.

## Project Overview

The vehicle completes two laps of a classroom-scale track. During the first lap, it follows the left wall and records a parking-space candidate detected on its right. On the second lap, it confirms the space, aligns with it, and executes a front-in parking maneuver.

At the same time, a camera-based perception node detects traffic signs and sends commands to the driving controller. The vehicle can stop, yield, enter a reduced-speed playground zone, and return to its normal speed.

## Key Features

- Autonomous navigation using LiDAR-based wall following and PID steering control
- Parking-space detection from LiDAR distance transitions and odometry
- Two-lap localization and slot confirmation before parking
- State-machine-based front-in parking control
- Traffic sign recognition using HSV color segmentation and contour geometry
- Temporal filtering to reduce camera false positives
- Forward obstacle detection with stop-and-resume behavior
- ROS 2 communication between perception, planning, and vehicle-control components

## System Architecture

The project is organized around two ROS 2 nodes:

### `front_in_parking_node`

Controls navigation and parking. It consumes LiDAR scans, odometry, and sign detections, then publishes Ackermann steering and speed commands. Its state machine moves through exploration, slot confirmation, alignment, turning, straightening, and completion.

### `sign_detection_node`

Processes images from an Intel RealSense D435i camera. It identifies red and yellow signs from their color and shape, applies a sliding-window filter, and publishes confirmed detections to the parking controller.

| Input | Purpose |
|---|---|
| 2D LiDAR | Wall following, parking-space detection, and forward safety |
| RGB camera | STOP, YIELD, playground-zone, and end-zone detection |
| Wheel odometry | Distance tracking, lap detection, and parking alignment |

### Repository Layout

```text
parking_perception/
├── nodes/          # ROS 2 node entry points and message handling
├── control/        # Parking states and control helpers
└── perception/     # Camera-based sign-detection algorithms
launch/             # ROS 2 launch configuration
test/               # Package quality checks
```

## Technical Approach

**Navigation:** A two-beam PID controller estimates the vehicle's position relative to the left wall and adjusts its steering while driving around the track.

**Parking-space detection:** The controller monitors the right-side LiDAR range for a near-wall to open-space to near-wall sequence. Odometry is used to measure the opening and reject spaces outside the expected size range.

**Parking:** A finite-state controller confirms the recorded space on a later lap, moves the vehicle to an entry pose, executes a fixed-angle turn, and drives forward into the space.

**Sign perception:** OpenCV converts each camera frame to HSV, segments red and yellow regions, and classifies contours using polygon shape and fill ratio. Multiple detections across consecutive frames are required before a sign affects the vehicle.

## Tech Stack

- Python
- ROS 2 / `rclpy`
- OpenCV and NumPy
- F1TENTH platform
- Intel RealSense D435i camera
- 2D LiDAR and wheel odometry
- Ackermann steering control

## Build and Run

The package is designed for a ROS 2 F1TENTH workspace with the vehicle's LiDAR, camera, odometry, and VESC drivers already configured.

```bash
cd ~/f1tenth_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select parking_perception
source install/setup.bash
ros2 launch parking_perception project_launch.py
```

The launch file starts both the parking controller and sign-detection node. Control and perception parameters can be tuned in `launch/project_launch.py`.

