# mecanumbot_camera

A ROS 2 based camera, detection, and behavior-control stack for a mecanum-drive robot that targets Raspberry Pi hardware.

## Features

- Raspberry Pi camera streaming via `video_publisher` published on `/camera/image_raw`
- HSV ball detection (`ball_tracker_rgb`) that publishes `/detections/ball` plus mask/debug feeds
- TFLite SSD person detection (`people_detector`) that publishes `/detections/person`
- A fused visualization node (`overlay_fused`) subscribed to the main image and both detection topics
- Full robot behavior finite state machine (`behavior_manager`) that handles searching, tracking, fetching, owner finding, delivery, and idling

Launch the entire stack with:

```bash
ros2 launch mecanumbot_camera camera_and_detectors.launch.py
```

> **Note:** The legacy ONNX files in `models/` are not used. `people_detector` downloads the required TFLite SSD model at runtime.

---

## 1. Hardware & Software Requirements

### Supported hardware

- **Raspberry Pi 5** - Raspberry Pi OS, ROS 2 from source, `vision_msgs` and `vision_opencv` built from source
- **Raspberry Pi 3** - Ubuntu 22.04 with official ROS 2 binaries that already contain `vision_msgs` and `vision_opencv`

### ROS 2 requirements

Any full ROS 2 distribution (e.g. Humble) works as long as `vision_msgs` and `vision_opencv` (cv_bridge, image_transport) are present.

### Python environment

- Python 3.11 (tested on Raspberry Pi OS)
- NumPy **must** stay on the 1.x series (e.g. `numpy==1.26.*`)
- NumPy 2.x is incompatible with TensorFlow Lite, OpenCV, and the ROS 2 Python bindings used here

---

## 2. Project Structure

```text
mecanumbot_camera/
|-- config/
|   \-- params.yaml                  # All configurable parameters
|-- launch/
|   \-- camera_and_detectors.launch.py
|-- mecanumbot_camera/
|   |-- video_publisher.py
|   |-- ball_tracker_rgb.py
|   |-- people_detector.py
|   |-- overlay_fused.py
|   \-- behavior_manager.py
|-- models/                          # Only used for historical ONNX artifacts
\-- media/, resource/, config/, etc.
```

---

## 3. Node Architecture

### Active nodes (camera_and_detectors.launch.py)

- `/video_publisher`
- `/ball_tracker_rgb`
- `/people_detector`
- `/overlay_fused`
- `/behavior_manager`

### Topic graph

```text
video_publisher
  -> /camera/image_raw        (sensor_msgs/Image)
  -> /camera/camera_info      (sensor_msgs/CameraInfo)

ball_tracker_rgb
  -> /camera/image_mask       (sensor_msgs/Image)  HSV mask
  -> /camera/image_debug      (sensor_msgs/Image)  debug view
  -> /detections/ball         (vision_msgs/Detection2DArray)

people_detector
  -> /camera/people_debug     (sensor_msgs/Image)
  -> /detections/person       (vision_msgs/Detection2DArray)

overlay_fused
  -> /camera/fused_debug      (sensor_msgs/Image)
  <- /camera/image_raw
  <- /detections/ball
  <- /detections/person

behavior_manager
  <- /detections/ball
  <- /detections/person
  -> /cmd_vel                 (geometry_msgs/Twist)
  -> /gripper_controller/commands (std_msgs/Float64)

Common ROS infrastructure topics such as `/tf`, `/tf_static`, `/rosout`, `/parameter_events`,
`/clicked_point`, `/initialpose`, and `/goal_pose` are also present.
```

### Node summaries

#### video_publisher

Streams frames from the Raspberry Pi CSI camera through `PiCamReader`, converts them with `cv_bridge`, and publishes `/camera/image_raw` along with camera info. It is optimized for Pi camera hardware rather than a USB v4l2 device.

#### ball_tracker_rgb

Performs HSV based color segmentation to detect the tracked ball.

- **Key parameters:** `image_topic`, `mask_image_topic`, `debug_image_topic`, `det_ball_topic`,
  `hsv_lower`, `hsv_upper`, `median_ksize`, `morph_kernel`, `morph_iters`,
  `min_area_px`, `min_circularity`, and `use_enclosing_circle`.
- **Outputs:** `/detections/ball`, `/camera/image_mask`, `/camera/image_debug`.

#### people_detector

Runs a TensorFlow Lite SSD person detector. The first launch ensures that the model defined in
`people_detector.py` is downloaded and extracted inside `models/` before inference starts.

- **Key parameters:** `image_topic`, `det_topic`, `conf_threshold`, `infer_every_n`.
- **Outputs:** `/detections/person`, `/camera/people_debug`.

#### overlay_fused

Combines the raw camera feed with ball and person detections to provide a single debug overlay on `/camera/fused_debug`.

#### behavior_manager

Implements the finite state machine that manages robot behavior, including search, tracking, fetch, grasp, owner finding, delivery, and waiting for the next ball.

- **Inputs:** `/detections/ball`, `/detections/person`.
- **Outputs:** `/cmd_vel`, `/gripper_controller/commands`.
- **Notable parameters:** `image_width`, `fetch_enter_px`, `fetch_stop_px`, `REQUIRED_BALL_STABLE`,
  `REQUIRED_PERSON_STABLE`, `ball_lost_timeout`, `person_lost_timeout`,
  `Kp_rot`, `Kp_fwd`, `max_ang`, `max_lin`, `search_speed`,
  `grasp_duration`, `new_ball_absent_time`.

---

## 4. Configuration (config/params.yaml)

`camera_and_detectors.launch.py` loads `config/params.yaml`, which is organized into the following groups:

- `ball_tracker_rgb`: HSV thresholds, morphological operations, min area/circularity, and topic names.
- `people_detector`: input topic, detection topic, confidence threshold, inference throttling.
- `overlay_fused`: input topics and output overlay topic.
- `behavior_manager`: PID gains, geometric thresholds, timeouts, and FSM tuning (see code comments for full list).

Edit the YAML file to adapt the system to different lighting conditions, camera resolutions, or robot dynamics.

---

## 5. Building & Running

### 5.1 Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Fortuz/mecanumbot_camera.git
cd ..
```

### 5.2 Install dependencies (Ubuntu 22.04 or Raspberry Pi 3)

```bash
sudo apt update
sudo apt install \
  ros-<ros2_distro>-vision-msgs \
  ros-<ros2_distro>-vision-opencv \
  python3-pip
```

### 5.3 Force NumPy 1.x

```bash
pip3 install --upgrade "numpy<2.0"
```

### 5.4 Build

```bash
cd ~/ros2_ws
colcon build --packages-select mecanumbot_camera
source install/setup.bash
```

### 5.5 Run the stack

```bash
ros2 launch mecanumbot_camera camera_and_detectors.launch.py
```

### 5.6 Inspect nodes and topics

```bash
ros2 node list
ros2 topic list
```

---

## 6. Debug & Visualization

Useful debug topics:

- `/camera/image_mask`
- `/camera/image_debug`
- `/camera/people_debug`
- `/camera/fused_debug`

View them with:

```bash
ros2 run rqt_image_view rqt_image_view
```

RViz or Foxglove can also be used for visualization.

---

## 7. Performance Tuning (Raspberry Pi 3 / Raspberry Pi 5)

- Increase `infer_every_n` in `people_detector` to skip frames and gain FPS.
- Increase `conf_threshold` for more stable person detections.
- Reduce `median_ksize`, `morph_kernel`, and `morph_iters` in `ball_tracker_rgb` to save CPU.
- Lower camera resolution or FPS in `PiCamReader` if the CPU or ISP is overloaded.

---

## 8. Docker Support (outdated)

The repository contains legacy Dockerfiles that assumed a v4l2 camera, ONNX detectors, no behavior manager, and no TFLite model download. They no longer reflect the current stack.

To modernize Docker support you would need to:

1. Base the image on `ros:<ros2_distro>-ros-base` (ARM64).
2. Install Python dependencies with `numpy<2.0`.
3. Build the package with `colcon`.
4. Expose the Pi camera or `/dev/video*`.
5. Use `camera_and_detectors.launch.py` as the entrypoint.

Until such an update is completed, run the stack natively on Raspberry Pi OS or Ubuntu 22.04.

---

## 9. License

Add your preferred license text here (e.g. MIT, Apache-2.0, or proprietary).
