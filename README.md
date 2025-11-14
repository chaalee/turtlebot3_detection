# TurtleBot3 Person Detection System

YOLO-based person detection with distance-proportional buzzer alerts for visually impaired runner guidance.

## Features

- Real-time person detection using YOLOv8 Nano
- Distance estimation based on bounding box size
- Distance-proportional buzzer alerts (faster beeps when closer)
- Emergency stop when person < 1.5m
- ROS 2 Humble compatible

## Installation

### On TurtleBot3:
```bash
# Clone repository
cd ~/turtlebot3_ws/src
git clone https://github.com/YOUR_USERNAME/turtlebot3_detection.git

# Install dependencies
sudo apt install ros-humble-usb-cam ros-humble-cv-bridge
pip3 install ultralytics --break-system-packages

# Build
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_detection
source install/setup.bash
```

## Usage

### Test Buzzer:
```bash
ros2 run turtlebot3_detection test_buzzer
```

### Test Camera:
```bash
ros2 launch turtlebot3_detection camera_launch.py

# View in RViz2 (on remote PC):
ros2 run rviz2 rviz2
# Add topic: /camera/image_raw
```

### Run Detection System:
```bash
ros2 launch turtlebot3_detection detection_launch.py
```

### Monitor Distance:
```bash
ros2 topic echo /detection/distance
```

## Buzzer Behavior

| Distance | Beep Interval | Note | Status |
|----------|---------------|------|--------|
| < 0.5m   | 0.1s (10 Hz)  | B    | Emergency |
| 0.5-1.0m | 0.2s (5 Hz)   | B    | Danger |
| 1.0-1.5m | 0.4s (2.5 Hz) | A    | Stop Zone |
| 1.5-2.0m | 0.8s (1.25 Hz)| G    | Warning |
| 2.0-2.5m | 1.5s (0.67 Hz)| F    | Caution |
| > 2.5m   | Silent        | -    | Safe |

## Calibration

Update `focal_length` parameter in `launch/detection_launch.py` after calibration.

## Team

- Advanced Mobile Robots Project
- Chulalongkorn University
