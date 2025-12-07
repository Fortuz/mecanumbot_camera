# Installation Guide

Step-by-step instructions for bringing up `mecanumbot_camera` on Raspberry Pi hardware (Pi 5 or Pi 3) or a desktop ROS 2 workstation.

The guide assumes **Ubuntu 22.04 + ROS2 Humble**.

---

## 2. Create a ROS 2 workspace and clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Fortuz/mecanumbot_camera.git
cd ..
```

If you plan to edit the package, keep the repo under version control inside `~/ros2_ws/src`.

---

## 3. Build the package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mecanumbot_camera
source install/setup.bash
```

For iterative development you can use `colcon build --symlink-install`.

---

## 4. Launch options

### Full stack

```bash
ros2 launch mecanumbot_camera camera_and_detectors.launch.py
```

This starts the following nodes:
- `video_publisher` (PiCamReader) - publishes `/camera/image_raw`
- `ball_tracker_rgb`
- `people_detector`
- `overlay_fused`
- `behavior_manager`

### Using an existing camera source

If your robot already publishes `/camera/image_raw`, run only the detection/behavior nodes and remap their input topics:

```bash
ros2 run mecanumbot_camera ball_tracker_rgb \
  --ros-args -p image_topic:=/your/camera/image_raw
```

Launch the other nodes similarly:

```bash
ros2 run mecanumbot_camera people_detector \
  --ros-args -p image_topic:=/your/camera/image_raw

ros2 run mecanumbot_camera overlay_fused \
  --ros-args -p image_topic:=/your/camera/image_raw
```

Alternatively, duplicate `launch/camera_and_detectors.launch.py`, remove the `video_publisher` block, and adjust the parameters before launching.

### Playing back sample media

Use the videos under `media/` when hardware is unavailable. Publish them to `/camera/image_raw` using your preferred tool (e.g., `ros2 bag play`, `image_publisher`, or a custom script) before launching the stack.

---

## 5. Verifying

After the stack starts:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /detections/ball --once
ros2 topic echo /detections/person --once
```

Visualize debug feeds with RViz or `rqt_image_view`:

```bash
ros2 run rqt_image_view rqt_image_view
```

Add `/camera/image_debug`, `/camera/people_debug`, or `/camera/fused_debug` to confirm the overlay is active.

---

## 6. Troubleshooting

| Symptom | Checks |
|---------|--------|
| `/camera/image_raw` missing | Ensure PiCamReader has access to the CSI camera. On Pi 3 + Ubuntu, use a USB camera or external driver. |
| No `/detections/ball` | Tune HSV thresholds (`ros2 param set /ball_tracker_rgb hsv_lower "[H,S,V]"`), adjust lighting, verify mask on `/camera/image_mask`. |
| No `/detections/person` | Confirm `tflite-runtime` installed, check network connectivity for first model download, raise `infer_every_n` to reduce CPU load. |
| Behavior manager not moving robot | Confirm `/cmd_vel` and `/gripper_controller/commands` are connected to the base/gripper drivers. Use `ros2 topic echo` to inspect. |
| Docker X11 issues on Windows | Ensure VcXsrv/x410 runs, set `DISPLAY=host.docker.internal:0.0`, allow public access in the X server settings. |

---

## 7. Clean rebuild / uninstall

To wipe build artifacts:

```bash
cd ~/ros2_ws
rm -rf build install log
```

Delete the repository if desired:

```bash
rm -rf ~/ros2_ws/src/mecanumbot_camera
```

---

## 8. Next steps

- Adjust parameters in `config/params.yaml` (ball tracker HSV bounds, people detector topics, overlay inputs).
- Tune FSM constants inside `mecanumbot_camera/behavior_manager.py` if your robot dimensions differ.
- Explore the Docker workflow in `README.MD` if you prefer containerized development on Windows.
