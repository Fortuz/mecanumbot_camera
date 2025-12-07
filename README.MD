# mecanumbot_camera

ROS 2 alapú kamera + detektálás + viselkedés-vezérlés stack egy Mecanum robothoz.  
A rendszer Raspberry Pi-re van optimalizálva, és a következőket tudja:

- Raspberry Pi kamera stream (`video_publisher` → `/camera/image_raw`)
- HSV-alapú labda detektálás (`ball_tracker_rgb` → `/detections/ball`)
- TFLite SSD alapú személydetektálás (`people_detector` → `/detections/person`)
- Egyesített debug overlay (`overlay_fused` → `/camera/fused_debug`)
- Viselkedés-kezelés (`behavior_manager`):  
  - labda keresés, követés, felvétel  
  - “gazda” (személy) felkutatása  
  - labda odaszállítása a személyhez  
  - új labda várása, új kör indítása

A teljes stacket a `camera_and_detectors.launch.py` launch fájl indítja.

---

## 1. Hardver és szoftver követelmények

### Cél hardverek

- **Raspberry Pi 5**
  - Tesztelt: Raspberry Pi OS, ROS 2 saját forrásból fordítva
- **Raspberry Pi 3**
  - Cél: Ubuntu 22.04 + gyári ROS 2 disztribúció  
    (itt a `vision_msgs` és `vision_opencv` csomagok már a disztribúció részei)

### ROS 2

- Bármely teljes értékű ROS 2 disztribúció használható (pl. Humble),  
  a továbbiakban `<ros2_distro>`-val hivatkozunk rá.
- Feltétel: legyen telepítve:
  - `vision_msgs`
  - `vision_opencv` (cv_bridge, image_transport stb.)

### Python környezet

- **Python:** 3.11 (Pi OS-en tesztelve)
- **Fontos:** a rendszer **NumPy 1.x**-re van hangolva  
  - pl. `numpy==1.26.*`
  - a NumPy 2.x verziók kompatibilitási problémákat okozhatnak (TFLite / OpenCV / ROS2 kombó).

---

## 2. Projekt felépítése

A ROS 2 package neve: `mecanumbot_camera`.

Főbb elemek:

- `mecanumbot_camera/launch/camera_and_detectors.launch.py`  
  – a *jelenleg használt* launch file
- `mecanumbot_camera/config/params.yaml`  
  – paraméterek (HSV, küszöbök, topic nevek, stb.)
- `mecanumbot_camera/mecanumbot_camera/`  
  - `video_publisher.py`
  - `ball_tracker_rgb.py` (vagy `ball_tracker_rgb_optimized.py`)
  - `people_detector.py`
  - `overlay_fused.py`
  - `behavior_manager.py`

> Megjegyzés: a `models/` mappa ONNX fájljai jelenleg **nem** használtak,  
> a `people_detector` TFLite modellt **futásidőben tölti le**.

---

## 3. Node-architektúra

### Futó node-ok (camera_and_detectors.launch.py)

- `/video_publisher`
- `/ball_tracker_rgb`
- `/people_detector`
- `/overlay_fused`
- `/behavior_manager`

### Topic-háló

```text
video_publisher
  ├─ /camera/image_raw (sensor_msgs/Image)
  └─ /camera/camera_info (sensor_msgs/CameraInfo)

ball_tracker_rgb
  ├─ /camera/image_mask   (sensor_msgs/Image)    – HSV maszk
  ├─ /camera/image_debug  (sensor_msgs/Image)    – debug kép
  └─ /detections/ball     (vision_msgs/Detection2DArray)

people_detector
  ├─ /camera/people_debug (sensor_msgs/Image)    – debug overlay
  └─ /detections/person   (vision_msgs/Detection2DArray)

overlay_fused
  ├─ /camera/fused_debug  (sensor_msgs/Image)    – labda + személy overlay
  └─ (input: /detections/ball, /detections/person, /camera/image_raw)

behavior_manager
  ├─ input: /detections/ball (vision_msgs/Detection2DArray)
  ├─ input: /detections/person (vision_msgs/Detection2DArray)
  ├─ output: /cmd_vel (geometry_msgs/Twist)
  └─ output: /gripper_controller/commands (std_msgs/Float64)
