#!/usr/bin/env python3
import os
import urllib.request
import zipfile
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from tflite_runtime.interpreter import Interpreter
try:
    from tflite_runtime.interpreter import load_delegate
except ImportError:
    load_delegate = None

# Ebben a TFLite SSD modellben a "person" class id = 0
PERSON_CLASS_ID = 0

MODEL_URL = "https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip"
MODEL_DIR = os.path.join(os.path.expanduser("~"), "tflite_models")
MODEL_FILE = f"{MODEL_DIR}/detect.tflite"


class PeopleDetectorTFLite(Node):
    def __init__(self):
        super().__init__("people_detector_tflite")

        self.bridge = CvBridge()

        # ---------------- Params ----------------
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("det_topic", "/detections/person")
        self.declare_parameter("conf_threshold", 0.65)
        self.declare_parameter("infer_every_n", 3)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/camera/people_debug")
        self.declare_parameter("model_path", "")
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("use_gpu_if_available", True)
        self.declare_parameter("gpu_delegate_lib", "")
        self.declare_parameter(
            "gpu_delegate_candidates",
            [
                "libtensorflowlite_gpu_delegate.so",
                "libtensorflowlite_gpu_delegate.so.2",
                "/usr/lib/aarch64-linux-gnu/libtensorflowlite_gpu_delegate.so",
                "/usr/local/lib/libtensorflowlite_gpu_delegate.so",
            ],
        )

        self.image_topic = self.get_parameter("image_topic").value
        self.det_topic = self.get_parameter("det_topic").value
        self.conf_thr = float(self.get_parameter("conf_threshold").value)
        self.infer_every_n = int(self.get_parameter("infer_every_n").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.model_path_param = self.get_parameter("model_path").value
        self.num_threads = int(self.get_parameter("num_threads").value)
        self.use_gpu_if_available = bool(self.get_parameter("use_gpu_if_available").value)
        self.gpu_delegate_lib = self.get_parameter("gpu_delegate_lib").value
        self.gpu_delegate_candidates = list(self.get_parameter("gpu_delegate_candidates").value)

        # ---------------- Publisher ----------------
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)
        self.pub_dbg = (
            self.create_publisher(Image, self.debug_image_topic, 10)
            if self.publish_debug_image
            else None
        )

        # ---------------- Subscriber ----------------
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.frame_idx = 0

        # Időbeli simítás + bbox megtartás gyorsabb streamre
        self.prev_person_box = None      # (x1, y1, x2, y2)
        self.smooth_alpha = 0.85         # 0.8–0.9: stabil, de nem túl laggos
        self.hold_frames = 0
        self.hold_max = 12                # 8-12

        # ---------------- Load TFLite model ----------------
        try:
            model_path = self.resolve_model_path()
        except Exception as e:
            self.get_logger().error(f"Model download/load failed: {e}")
            self.interpreter = None
            return

        self.get_logger().info(f"Using model: {model_path}")
        self.interpreter = self.create_interpreter(model_path)
        if self.interpreter is None:
            return

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        # Standard SSD model output:
        # 0: detection_boxes (1,100,4)
        # 1: detection_classes (1,100)
        # 2: detection_scores  (1,100)
        # 3: num_detections    (1)
        self.idx_boxes = output_details[0]["index"]
        self.idx_classes = output_details[1]["index"]
        self.idx_scores = output_details[2]["index"]

        self.input_index = input_details[0]["index"]
        self.input_height = input_details[0]["shape"][1]
        self.input_width = input_details[0]["shape"][2]

    def create_interpreter(self, model_path: str):
        if self.use_gpu_if_available:
            if load_delegate is None:
                self.get_logger().warn("GPU delegate requested, but load_delegate is unavailable; falling back to CPU.")
            else:
                candidate_libs = []
                if self.gpu_delegate_lib:
                    candidate_libs.append(self.gpu_delegate_lib)
                candidate_libs.extend([lib for lib in self.gpu_delegate_candidates if lib])

                # Keep order while removing duplicates.
                candidate_libs = list(dict.fromkeys(candidate_libs))

                for delegate_lib in candidate_libs:
                    try:
                        delegate = load_delegate(delegate_lib)
                        interpreter = Interpreter(
                            model_path=model_path,
                            num_threads=self.num_threads,
                            experimental_delegates=[delegate],
                        )
                        interpreter.allocate_tensors()
                        self.get_logger().info(f"Using TFLite GPU delegate: {delegate_lib}")
                        return interpreter
                    except Exception as e:
                        self.get_logger().warn(
                            f"GPU delegate load failed ({delegate_lib}): {e}. Trying next option."
                        )

                self.get_logger().warn("No working TFLite GPU delegate found; using CPU.")

        try:
            interpreter = Interpreter(model_path=model_path, num_threads=self.num_threads)
            interpreter.allocate_tensors()
            self.get_logger().info(f"Using CPU inference with num_threads={self.num_threads}.")
            return interpreter
        except Exception as e:
            self.get_logger().error(f"CPU interpreter init failed: {e}")
            self.interpreter = None
            return None

    # ==========================================================
    def on_image(self, msg: Image):
        self.frame_idx += 1

        dets_msg = Detection2DArray()
        dets_msg.header = msg.header

        if self.interpreter is None:
            self.pub_det.publish(dets_msg)
            return

        # Ha minden N-edik frame-en inferelünk
        if self.frame_idx % self.infer_every_n != 0:
            self.pub_det.publish(dets_msg)
            return

        # -------- Read image --------
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            self.pub_det.publish(dets_msg)
            return

        h, w = img.shape[:2]

        # -------- Preprocess --------
        #resized = cv2.resize(img, (self.input_width, self.input_height))
        resized = cv2.resize(img, (self.input_width, self.input_height), interpolation=cv2.INTER_NEAREST)
        #rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        rgb = resized[:, :, ::-1]
        inp = np.expand_dims(rgb, axis=0)

        # -------- Inference --------
        self.interpreter.set_tensor(self.input_index, inp)
        self.interpreter.invoke()

        boxes = self.interpreter.get_tensor(self.idx_boxes)[0]
        classes = self.interpreter.get_tensor(self.idx_classes)[0]
        scores = self.interpreter.get_tensor(self.idx_scores)[0]

        detections = []

        for i in range(len(scores)):
            sc = float(scores[i])
            if sc < self.conf_thr:
                continue
            if int(classes[i]) != PERSON_CLASS_ID:
                continue

            ymin, xmin, ymax, xmax = boxes[i]
            x1 = int(xmin * w)
            y1 = int(ymin * h)
            x2 = int(xmax * w)
            y2 = int(ymax * h)

            detections.append((x1, y1, x2, y2, sc))

        # -------- Handle detections & hold logic --------
        if len(detections) > 0:
            # VAN új detekció → reset hold + válasszuk a legjobbat
            self.hold_frames = 0
            x1, y1, x2, y2, sc = max(detections, key=lambda d: d[4])
        else:
            # NINCS új detekció
            if self.prev_person_box is not None and self.hold_frames < self.hold_max:
                # Tartsuk meg az előző boxot (kitöltjük a gyorsabb streamet)
                x1, y1, x2, y2 = self.prev_person_box
                sc = 0.0  # nincs friss score, de a bbox marad
                self.hold_frames += 1

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float((x1 + x2) / 2.0)
                det.bbox.center.position.y = float((y1 + y2) / 2.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score = float(sc)
                det.results.append(hyp)

                dets_msg.detections.append(det)

                self.pub_det.publish(dets_msg)
                if self.publish_debug_image and self.pub_dbg is not None:
                    dbg = img.copy()
                    cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
                return
            
            else:
                # Már túl régi, töröljük a boxot, üres frame-et küldünk
                self.prev_person_box = None
                self.hold_frames = 0

                # debug: eredeti kép bbox nélkül
                if self.publish_debug_image and self.pub_dbg is not None:
                    self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                self.pub_det.publish(dets_msg)
                return

        # -------- Temporal smoothing --------
        if self.prev_person_box is not None:
            px1, py1, px2, py2 = self.prev_person_box
            a = self.smooth_alpha
            x1 = a * x1 + (1.0 - a) * px1
            y1 = a * y1 + (1.0 - a) * py1
            x2 = a * x2 + (1.0 - a) * px2
            y2 = a * y2 + (1.0 - a) * py2

        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)

        self.prev_person_box = (x1, y1, x2, y2)

        # -------- ROS message --------
        det = Detection2D()
        det.header = msg.header
        det.bbox.center.position.x = float((x1 + x2) / 2.0)
        det.bbox.center.position.y = float((y1 + y2) / 2.0)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = "person"
        hyp.hypothesis.score = float(sc)
        det.results.append(hyp)

        dets_msg.detections.append(det)

        # debug image
        self.pub_det.publish(dets_msg)
        if self.publish_debug_image and self.pub_dbg is not None:
            dbg = img.copy()
            cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))

    def resolve_model_path(self):
        if self.model_path_param:
            expanded = os.path.expanduser(self.model_path_param)
            if os.path.exists(expanded):
                return expanded
            self.get_logger().warn(
                f"Configured model_path does not exist: {expanded}. Falling back to auto-download."
            )
        return self.download_model_if_needed()

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

        # find detect.tflite
        for root, dirs, files in os.walk(MODEL_DIR):
            for f in files:
                if f.endswith(".tflite"):
                    return os.path.join(root, f)

        raise FileNotFoundError("detect.tflite not found after extraction.")


def main():
    rclpy.init()
    node = PeopleDetectorTFLite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
