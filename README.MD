# mecanumbot_camera

A ROS 2–based camera, detection, and behavior-control stack designed for a Mecanum robot, optimized for Raspberry Pi hardware.

The system provides:

- Raspberry Pi camera streaming (`video_publisher` → `/camera/image_raw`)
- HSV-based ball detection (`ball_tracker_rgb` → `/detections/ball`)
- TFLite SSD person detection (`people_detector` → `/detections/person`)
- Fused debug overlay (`overlay_fused` → `/camera/fused_debug`)
- Full behavior FSM (`behavior_manager`) for:
  - ball search, tracking, fetching
  - finding the “owner” (a detected person)
  - delivering the ball
  - waiting for a new ball to start the next cycle

The entire stack is launched via:
ros2 launch mecanumbot_camera camera_and_detectors.launch.py

yaml
Kód másolása

---

## 1. Hardware & Software Requirements

### Supported hardware

#### **Raspberry Pi 5**
- Tested using Raspberry Pi OS  
- ROS 2 built from source  
- Additional packages built from source (`vision_msgs`, `vision_opencv`)

#### **Raspberry Pi 3**
- Target platform: **Ubuntu 22.04 + official ROS 2 binaries**
- Required packages (`vision_msgs`, `vision_opencv`) are included in the distribution.

---

### ROS 2 Requirements

Any full ROS 2 distribution (e.g., Humble) is supported.

Required packages:
- `vision_msgs`
- `vision_opencv` (cv_bridge, image_transport)

---

### Python Environment

- **Python 3.11** (tested on Raspberry Pi OS)
- **IMPORTANT:** The system requires **NumPy 1.x**  
  (Example: `numpy==1.26.*`)

NumPy 2.x is **not supported** due to compatibility issues with:
- TFLite
- OpenCV
- ROS 2 bindings

---

## 2. Project Structure

Package name: `mecanumbot_camera`

Key directories and files:

mecanumbot_camera/
├── launch/
│ └── camera_and_detectors.launch.py # Main launch file
├── config/
│ └── params.yaml # All configurable parameters
└── mecanumbot_camera/
├── video_publisher.py
├── ball_tracker_rgb.py
├── people_detector.py
├── overlay_fused.py
└── behavior_manager.py

yaml
Kód másolása

> Note:  
> ONNX files in `models/` are **not used**.  
> The `people_detector` downloads a **TFLite SSD model at runtime**.

---

## 3. Node Architecture

### Active nodes (from `camera_and_detectors.launch.py`)

/video_publisher
/ball_tracker_rgb
/people_detector
/overlay_fused
/behavior_manager

bash
Kód másolása

---

### Topic Graph

```text
video_publisher
  ├─ /camera/image_raw      (sensor_msgs/Image)
  └─ /camera/camera_info    (sensor_msgs/CameraInfo)

ball_tracker_rgb
  ├─ /camera/image_mask     (sensor_msgs/Image)  – HSV mask
  ├─ /camera/image_debug    (sensor_msgs/Image)  – debug view
  └─ /detections/ball       (vision_msgs/Detection2DArray)

people_detector
  ├─ /camera/people_debug   (sensor_msgs/Image)
  └─ /detections/person     (vision_msgs/Detection2DArray)

overlay_fused
  ├─ /camera/fused_debug    (sensor_msgs/Image)
  └─ inputs: /detections/ball, /detections/person, /camera/image_raw

behavior_manager
  ├─ inputs:
  │    /detections/ball     (vision_msgs/Detection2DArray)
  │    /detections/person   (vision_msgs/Detection2DArray)
  ├─ outputs:
       /cmd_vel                    (geometry_msgs/Twist)
       /gripper_controller/commands (std_msgs/Float64)
Additional common ROS topic providers:

/tf, /tf_static

/parameter_events, /rosout

/clicked_point, /initialpose, /goal_pose

4. Node Descriptions
4.1. video_publisher
Function:
Streams frames from the Raspberry Pi camera using a PiCamReader.

python
Kód másolása
class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cam = PiCamReader(width=640, height=480, fps=30)
        self.timer = self.create_timer(1/30.0, self.tick)
Input: Raspberry Pi CSI camera

Output:

/camera/image_raw

/camera/camera_info (if implemented)

Note:
This node uses PiCam, not a v4l2 USB camera.

4.2. ball_tracker_rgb
Function:
Detects a colored (HSV domain) ball in the incoming image.

Key parameters from params.yaml:
image_topic

mask_image_topic

debug_image_topic

det_ball_topic

hsv_lower, hsv_upper

median_ksize

morph_kernel, morph_iters

min_area_px

min_circularity

use_enclosing_circle

Outputs:
/detections/ball — vision_msgs/Detection2DArray

/camera/image_mask (debug)

/camera/image_debug (debug)

4.3. people_detector
Function:
Performs person detection using a downloaded TFLite SSD model.

Runtime model download:
python
Kód másolása
def download_model_if_needed(self):
    os.makedirs(MODEL_DIR, exist_ok=True)

    if os.path.exists(MODEL_FILE):
        return MODEL_FILE

    zip_path = f"{MODEL_DIR}/model.zip"
    urllib.request.urlretrieve(MODEL_URL, zip_path)
    with zipfile.ZipFile(zip_path, "r") as z:
        z.extractall(MODEL_DIR)

    for root, dirs, files in os.walk(MODEL_DIR):
        for f in files:
            if f.endswith(".tflite"):
                return os.path.join(root, f)

    raise FileNotFoundError("detect.tflite not found after extraction.")
Model URL is defined in people_detector.py.

Parameters:
image_topic

det_topic

conf_threshold

infer_every_n (performance tuning)

Outputs:
/detections/person

/camera/people_debug

4.4. overlay_fused
Function:
Combines ball + person detections into a single debug overlay.

Inputs:

/camera/image_raw

/detections/ball

/detections/person

Output:

/camera/fused_debug

4.5. behavior_manager
Function:
A complete FSM managing robot behavior:

ball search

ball following

ball grasp

owner search

delivery

waiting for new ball

Inputs:
/detections/ball

/detections/person

Outputs:
/cmd_vel

/gripper_controller/commands

Core parameters:
image_width = 640

Ball thresholds:

fetch_enter_px = 200

fetch_stop_px = 280

Stability:

REQUIRED_BALL_STABLE = 3

REQUIRED_PERSON_STABLE = 3

Timeouts:

ball_lost_timeout = 1.0

person_lost_timeout = 1.0

Control gains:

Kp_rot = 0.0025

Kp_fwd = 0.015

max_ang = 0.8

max_lin = 0.30

search_speed = 0.25

Delivery:

grasp_duration = 1.0

new_ball_absent_time = 2.0

FSM States:
SEARCH

TRACK_BALL

FETCH

GRASP

FIND_OWNER

DELIVER

Each transition is fully described inside behavior_manager.py.

5. Configuration (params.yaml)
The launch file loads:

arduino
Kód másolása
config/params.yaml
Parameter groups:
ball_tracker_rgb
hsv_lower, hsv_upper

image_topic, mask_image_topic, debug_image_topic

morph_kernel, morph_iters, median_ksize

min_area_px, min_circularity

det_ball_topic

people_detector
image_topic

det_topic

conf_threshold

infer_every_n

overlay_fused
image_topic

ball_det_topic

person_det_topic

out_topic

Refer to config/params.yaml for default values.

6. Building & Running
6.1. Clone the repository
bash
Kód másolása
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Fortuz/mecanumbot_camera.git
cd ..
6.2. Install dependencies (Ubuntu 22.04 / Pi 3)
bash
Kód másolása
sudo apt update
sudo apt install \
  ros-<ros2_distro>-vision-msgs \
  ros-<ros2_distro>-vision-opencv \
  python3-pip
Force NumPy 1.x:

bash
Kód másolása
pip3 install --upgrade "numpy<2.0"
6.3. Build
bash
Kód másolása
cd ~/ros2_ws
colcon build --packages-select mecanumbot_camera
source install/setup.bash
6.4. Run
bash
Kód másolása
ros2 launch mecanumbot_camera camera_and_detectors.launch.py
Expected nodes:

bash
Kód másolása
ros2 node list
Expected topics:

bash
Kód másolása
ros2 topic list
7. Debug & Visualization
Debug image topics:

/camera/image_mask

/camera/image_debug

/camera/people_debug

/camera/fused_debug

View via:

bash
Kód másolása
ros2 run rqt_image_view rqt_image_view
or RViz / Foxglove.

8. Performance Tuning (Pi 3 / Pi 5)
People detector
Increase infer_every_n → fewer inferences, higher FPS

Increase conf_threshold → more stable detections

Ball tracker
Reduce median_ksize, morph_kernel, morph_iters for speed

Reduce camera resolution or FPS in PiCamReader

9. Docker Support (OUTDATED, needs update)
The repository contains older Dockerfiles that assume:

v4l2 camera

ONNX-based detectors

No behavior manager

No TFLite runtime model download

These no longer reflect the current stack.

To update Docker support:

Base on ros:<ros2_distro>-ros-base (ARM64).

Install Python deps + numpy<2.0.

Build the package with colcon.

Expose PiCam or /dev/video*.

Set launch file as entrypoint.

Until updated, native execution on Pi OS / Ubuntu 22.04 is recommended.

10. License
(Fill in the license for your project, for example MIT or Apache-2.0.)

