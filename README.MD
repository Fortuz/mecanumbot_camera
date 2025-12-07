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


Ehhez kapcsolódó egyéb standard topicok:

/tf, /tf_static, /parameter_events, /rosout, stb.

/clicked_point, /initialpose, /goal_pose (más stack-elemekkel való integrációhoz)

4. Node-onkénti leírás
4.1. video_publisher

Funkció:
Raspberry Pi kameráról (PiCam) olvas képet, és publikálja /camera/image_raw topicra.

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cam = PiCamReader(width=640, height=480, fps=30)

        self.timer = self.create_timer(1/30.0, self.tick)


Bemenet: PiCam (CSI) – PiCamReader(width=640, height=480, fps=30)

Kimenet:

/camera/image_raw (sensor_msgs/Image)

/camera/camera_info (ha implementálva van a node-ban)

Megjegyzés: jelenleg nem v4l2 kamera, hanem közvetlen Pi kamera használat.

4.2. ball_tracker_rgb

Funkció:
HSV-tartomány alapú labda detektálás a /camera/image_raw képeken.

Fontos paraméterek:

image_topic – bemeneti képtopic (pl. /camera/image_raw)

mask_image_topic – HSV maszk képtopic (pl. /camera/image_mask)

debug_image_topic – debug overlay (pl. /camera/image_debug)

det_ball_topic – detekciók kimenete (pl. /detections/ball)

hsv_lower, hsv_upper – HSV tartomány a labdára

median_ksize – median blur kernel méret

morph_kernel, morph_iters – morfológiai szűrés paraméterei

min_area_px – minimális kontúr terület (zajszűrés)

min_circularity – körkörösség küszöb

use_enclosing_circle – köralakú becslés használata

Kimenet:

/detections/ball – vision_msgs/Detection2DArray

1 detekció / frame (labda), bounding box + confidence

/camera/image_mask – HSV maszk (debug)

/camera/image_debug – input kép + overlayelt bounding box (debug)

4.3. people_detector

Funkció:
Személy detektálása TFLite SSD modellel egy képkockán.

Modell letöltés:

A node nem a models/ mappából használ modellt, hanem futásidőben tölti le:

def download_model_if_needed(self):
    os.makedirs(MODEL_DIR, exist_ok=True)

    if os.path.exists(MODEL_FILE):
        return MODEL_FILE

    zip_path = f"{MODEL_DIR}/model.zip"
    self.get_logger().info(f"Downloading TFLite SSD model...")
    urllib.request.urlretrieve(MODEL_URL, zip_path)
    self.get_logger().info(f"Extracting model...")
    with zipfile.ZipFile(zip_path, "r") as z:
        z.extractall(MODEL_DIR)

    for root, dirs, files in os.walk(MODEL_DIR):
        for f in files:
            if f.endswith(".tflite"):
                return os.path.join(root, f)

    raise FileNotFoundError("detect.tflite not found after extraction.")


A pontos MODEL_URL a kódban van definiálva (people_detector.py).

A ZIP-ből egy *.tflite modell kerül elő, ezt használja inferenciára.

Paraméterek:

image_topic – bemeneti képek topicja (pl. /camera/image_raw)

det_topic – kimeneti detekciós topic (pl. /detections/person)

conf_threshold – detekció confidence küszöb

infer_every_n – minden n-edik frame-en fut csak inference (Pi CPU kímélése)

Kimenet:

/detections/person – vision_msgs/Detection2DArray

azonosított személy(ek) bounding box-szal

/camera/people_debug – debug kép (eredeti kép + bounding boxok)

4.4. overlay_fused

Funkció:
A labda és személy detekciók egyesítése egy debug overlay képre.

Bemenet:

image_topic – tipikusan /camera/image_raw

ball_det_topic – /detections/ball

person_det_topic – /detections/person

Kimenet:

out_topic – /camera/fused_debug (annotált kép, mindkét detekcióval)

4.5. behavior_manager

Funkció:
Magas szintű viselkedéskezelés (FSM) a labda → felvétel → gazda megkeresése → labda átadása folyamatra.

Input topicok:

/detections/ball – vision_msgs/Detection2DArray

/detections/person – vision_msgs/Detection2DArray

Output topicok:

/cmd_vel – geometry_msgs/Twist (mozgás vezérlés)

/gripper_controller/commands – std_msgs/Float64 (gripper nyitás/zárás)

Főbb belső paraméterek:

image_width = 640

Ball thresholds – hiszterézis:

fetch_enter_px = 200 – TRACK → FETCH

fetch_stop_px = 280 – FETCH → GRASP

Stabilitási követelmények:

REQUIRED_BALL_STABLE = 3

REQUIRED_PERSON_STABLE = 3

Time-outok:

ball_lost_timeout = 1.0

person_lost_timeout = 1.0

Vezérlő paraméterek:

Kp_rot = 0.0025

Kp_fwd = 0.015

max_ang = 0.8

max_lin = 0.30

search_speed = 0.25

Személy küszöb:

owner_threshold_px = 150

Grasp:

grasp_duration = 1.0 s

Új labda felismerés:

new_ball_absent_time = 2.0 s

Állapotgép (FSM) állapotai:

SEARCH

A robot forog (twist.angular.z = search_speed), és labdát keres.

Ha egy korábbi delivery után új kört indít:

megvárja, amíg a labda eltűnik new_ball_absent_time ideig

Ha stabilan lát labdát (REQUIRED_BALL_STABLE frame), átmegy TRACK_BALL állapotba.

TRACK_BALL

Labdára áll rá, forgat + előre mozog (compute_ball_control()).

Ha a labda szélessége eléri a fetch_enter_px értéket és stabil, átmegy FETCH állapotba.

Ha elveszíti a labdát → vissza SEARCH.

FETCH

Finom közelítés a labdához (compute_ball_control(slow=True)).

Ha a labda szélessége ≥ fetch_stop_px és stabil,

megállítja a robotot (cmd_pub.publish(Twist()))

állapot: GRASP

GRASP

Lezárja a gripper-t, vár grasp_duration ideig.

Utána → FIND_OWNER, kinullázva a személy-stabilitást.

FIND_OWNER

Személyt keres (/detections/person).

Minimum min_find_owner_time ideig ebben az állapotban marad.

Ha személyt lát: compute_person_control()-lal közelít.

Ha a személy szélessége ≥ owner_threshold_px és stabil (REQUIRED_PERSON_STABLE),
→ DELIVER.

DELIVER

Megáll, kinyitja a gripper-t (open_gripper()).

deliver_start_time után kb. 2 s-ig marad itt.

Utána → vissza SEARCH

beállítja waiting_for_new_ball = True,

új labdát csak akkor fogad el, ha közben egy ideig nem látott labdát.

5. Konfiguráció (params.yaml)

A camera_and_detectors.launch.py az mecanumbot_camera/config/params.yaml fájlt tölti be:

PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),


A params.yaml tartalmazza többek között:

ball_tracker_rgb:

image_topic

debug_image_topic

mask_image_topic

det_ball_topic

hsv_lower, hsv_upper

min_area_px, min_circularity

median_ksize, morph_kernel, morph_iters

use_enclosing_circle

people_detector:

image_topic

det_topic

conf_threshold

infer_every_n

overlay_fused:

image_topic

ball_det_topic

person_det_topic

out_topic

A pontos default értékekért nézd meg a config/params.yaml-t.

6. Build és futtatás
6.1. Forráskód elhelyezése
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Fortuz/mecanumbot_camera.git
cd ..

6.2. Függőségek (Ubuntu 22.04 / Pi 3, gyári ROS2)
sudo apt update
sudo apt install \
  ros-<ros2_distro>-vision-msgs \
  ros-<ros2_distro>-vision-opencv \
  python3-pip


NumPy downgrade (ha nem 1.x):

pip3 install --upgrade "numpy<2.0"


(ha virtuális környezetet használsz, azt előtte aktiváld)

6.3. Build
cd ~/ros2_ws
colcon build --packages-select mecanumbot_camera
source install/setup.bash

6.4. Futtatás
ros2 launch mecanumbot_camera camera_and_detectors.launch.py


Elvárt futó node-ok:

ros2 node list
# /video_publisher
# /ball_tracker_rgb
# /people_detector
# /overlay_fused
# /behavior_manager


Elvárt fő topicok:

ros2 topic list
# /camera/image_raw
# /camera/camera_info
# /camera/image_mask
# /camera/image_debug
# /camera/people_debug
# /camera/fused_debug
# /detections/ball
# /detections/person
# /cmd_vel
# /gripper_controller/commands
# ...

7. Debug és vizualizáció

A debug képek a következő topicokon érhetők el:

/camera/image_mask – HSV maszk (labda)

/camera/image_debug – labda detektálás overlay

/camera/people_debug – személy detektálás overlay

/camera/fused_debug – labda + személy overlay

Megnézhetők pl.:

ros2 run rqt_image_view rqt_image_view


vagy bármely más image viewer-rel (RViz, Foxglove, stb.).

8. Teljesítményhangolás (Pi 3 / Pi 5)

people_detector:

infer_every_n növelése → gyorsabb futás, ritkább detektálás

conf_threshold emelése → kevesebb, de megbízhatóbb detektálás

ball_tracker_rgb:

median_ksize, morph_kernel, morph_iters csökkentése → kevesebb blur / morfológia, gyorsabb futás

Túl lassú / akadozó viselkedés esetén:

kép felbontás csökkentése (VideoPublisher / PiCamReader paraméterezés)

kamerafps csökkentése

9. Docker (jelenleg: frissítés szükséges)

A repo tartalmaz Dockerfile-okat a korábbi, teljesen konténeresített verzióhoz.
Fontos: ezek még a régi stackre vonatkoznak (v4l2 kamera, ONNX modellfájlok stb.),
és nem tükrözik a most Pi 5-ön futó konfigurációt:

PiCam alapú video_publisher

TFLite SSD dinamkus letöltése people_detector-ben

behavior_manager FSM integráció

A Docker támogatás frissítése során az alábbiakat kell majd megtenni:

Base image: ARM64-es ros:<ros2_distro>-ros-base image használata.

Python függőségek + numpy<2.0 fixálása.

A mecanumbot_camera package beépítése a konténerben (colcon build).

Kamera device (/dev/video* vagy PiCam) és szükséges udev/jogosultságok átadása.

A camera_and_detectors.launch.py legyen az entrypoint.

Amíg ez nincs kész, javasolt natív futtatás Pi OS / Ubuntu 22.04 alatt.

10. Licenc

(Töltsd ki a saját licenceddel – pl. Apache 2.0 / MIT, stb.)


---

Ha szeretnéd, a következő körben tudok hozzá írni egy **konkrét, Pi-re optimalizált Dockerfile**-t is (pl. `Dockerfile.pi5`), ami már ehhez a stackhez igazodik, és mellé egy minimál `docker-compose.yml`-t.
::contentReference[oaicite:0]{index=0}