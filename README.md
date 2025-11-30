# Autonomous Landing

ROS-based UAV Autonomous Landing System

## Overview

This package provides a ROS-based autonomous landing system for UAVs (Unmanned Aerial Vehicles) using camera vision and UKF (Unscented Kalman Filter) to automatically land on a helipad.

### Key Features

- **Helipad Detection**: Real-time helipad recognition using Hough Circle Transform
- **State Estimation**: UKF-based estimation of drone and target position/velocity
- **MAVROS Integration**: Communication with PX4 flight controller
- **LRF Sensor Support**: Altitude measurement via Laser Range Finder
- **Coordinate Transformation**: WGS-84 → ECEF → ENU coordinate conversion

## System Requirements

### Hardware
- PX4-based flight controller
- RGB camera (1920x1080 resolution recommended)
- Laser Range Finder (LRF)
- Gimbal mount (optional)

### Software
- ROS (Kinetic/Melodic/Noetic)
- Python 2.7+ or Python 3.x
- MAVROS
- OpenCV

### Python Dependencies
```bash
pip install numpy filterpy scipy opencv-python
```

## Installation

### 1. Setup Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clone Package
```bash
git clone https://github.com/lwcworld/autonomous_landing.git
```

### 3. Install Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### 1. Connect MAVROS

First, connect to the flight controller via MAVROS:
```bash
roslaunch mavros px4.launch
```

### 2. Run Helipad Detection Node

Run the node that detects the helipad from camera images:
```bash
rosrun autonomous_landing hough_circles.py
```

### 3. Run Main Autonomous Landing Node

Run the autonomous landing algorithm:
```bash
rosrun autonomous_landing main.py
```

## Node Description

### main.py
Main autonomous landing node that performs:
- UKF-based state estimation
- Landing condition assessment
- Gimbal mount control

### hough_circles.py
Detects circular helipad from camera images:
- Subscribes to `/camera/color/image_raw` topic
- Applies Hough Circle Transform
- Publishes detected position to `/target_pixel` topic

## ROS Topics

### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Drone position and attitude |
| `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | Drone velocity |
| `/mavros/px4flow/ground_distance` | `sensor_msgs/Range` | LRF distance measurement |
| `/target_pixel` | `geometry_msgs/PointStamped` | Detected helipad pixel coordinates |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Camera image |

### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | Velocity command |
| `/mavros/mount_control/command` | `mavros_msgs/MountControl` | Gimbal control command |
| `/helipad_marker` | `visualization_msgs/Marker` | RViz helipad visualization |
| `/ownship_marker` | `visualization_msgs/Marker` | RViz drone visualization |

## Parameter Configuration

The following parameters can be modified in `Variables.py`:

### Frequency Settings
- `freq_est`: State estimation frequency (default: 5 Hz)
- `freq_ctrl`: Control frequency (default: 20 Hz)
- `freq_rviz`: RViz visualization frequency (default: 10 Hz)

### Landing Threshold
- `th_L`: Landing probability threshold (default: 0.8)
- `th_viscen`: Target center threshold (default: 500 pixels)

### Camera Settings
- `cam_bound`: Camera resolution [height, width] (default: [1080, 1920])

## File Structure

```
autonomous_landing/
├── CMakeLists.txt          # CMake build configuration
├── package.xml             # ROS package manifest
├── README.md               # User manual
└── scripts/
    ├── main.py             # Main autonomous landing node
    ├── Algorithms.py       # UKF algorithm implementation
    ├── Variables.py        # State variables and parameters
    ├── Subscribers.py      # ROS subscriber class
    ├── Publishers.py       # ROS publisher class
    ├── Scenario.py         # Waypoint scenario
    ├── Utils.py            # Utility functions
    ├── hough_circles.py    # Helipad detection node
    └── coordinate_transform.py  # Coordinate transformation utilities
```

## Algorithm Description

### State Estimation (UKF)
Estimates a 15-dimensional state vector:
- Drone position (x, y, z) and velocity (vx, vy, vz)
- Drone attitude (roll, pitch, yaw) and angular rates
- Target position (x_t, y_t, z_t)

### Landing Condition Assessment
Switches to landing mode when the following conditions are met:
1. Vision measurement is up-to-date
2. LRF measurement is up-to-date
3. Drone state information is up-to-date
4. Target is centered in camera view

## Troubleshooting

### Helipad Not Detected
- Check camera topic name: `/camera/color/image_raw`
- Check lighting conditions
- Adjust Hough parameters in `hough_circles.py`

### MAVROS Connection Failed
- Check flight controller connection
- Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`

### UKF Divergence
- Adjust sensor noise parameters (`z_std_o`, `z_std_vis`, `z_std_r`)
- Adjust process noise (`Q_pos_o`, `Q_ang_o`, `Q_pos_t`)

## License

This project is open source.

## Contributing

Please submit bug reports and feature requests through GitHub Issues.
