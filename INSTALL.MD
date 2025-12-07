# Installation Guide

This document describes how to install and run the full mecanumbot_camera system, including:
- camera streaming,
- classical CV-based ball detection,
- TFLite-based person detection,
- the Behavior Manager FSM,
- robot base and gripper interaction.

The guide assumes **Ubuntu 22.04 + ROS2 Humble**.

---

## 1. System Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- OpenCV (for classical ball detection)
- tflite-runtime (for person detection)
- vision_msgs (ROS2 message type)
- A USB camera or real robot camera source

---

## 2. Clone and Build the Package

```bash
git clone https://github.com/Fortuz/mecanumbot_camera
cd mecanumbot_camera
colcon build --symlink-install
source install/setup.bash
3. Install Python Dependencies
OpenCV (ball detection)
bash
Kód másolása
pip install opencv-python
TFLite runtime (person detection)
For x86_64:

bash
Kód másolása
pip install tflite-runtime
For Jetson / ARM systems:

bash
Kód másolása
sudo apt-get install python3-tflite-runtime
4. Launching Components Individually
Camera
bash
Kód másolása
ros2 launch mecanumbot_camera camera.launch.py
Ball Detector (OpenCV)
bash
Kód másolása
ros2 run mecanumbot_camera ball_detector
Person Detector (TFLite)
bash
Kód másolása
ros2 run mecanumbot_camera person_detector
Behavior Manager FSM
bash
Kód másolása
ros2 run mecanumbot_camera behavior_manager
5. Running the Full Integrated System
bash
Kód másolása
ros2 launch mecanumbot_camera full_system.launch.py
This launch file starts:

camera node

ball detector

person detector

behavior manager

all required topics (/detections/ball, /detections/person, /cmd_vel, /gripper_controller/commands)

6. Troubleshooting
No camera detected
swift
Kód másolása
[ERROR] Cannot open /dev/video0
Check udev permissions

Try: sudo chmod 666 /dev/video0

No /detections/person messages
TFLite model path is incorrect

TFLite runtime missing

Person too small or not in field of view

No /detections/ball messages
Incorrect HSV threshold

Poor lighting conditions

Tune classical CV thresholds in ball_detector.py

Robot does not move
/cmd_vel namespace mismatch

Check ros2 topic echo /cmd_vel

Ensure base controller is running

7. Notes for Real-Robot Deployment
Use a USB camera with exposure locked for stable ball tracking

Ensure gripper calibration before running GRASP state

If latency is high, reduce image resolution in the camera node

8. Uninstall
To remove the workspace:

bash
Kód másolása
rm -rf install build log