#!/usr/bin/env python3
import os
import rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import traceback, sys, time

try:
    import onnxruntime as ort
except Exception as e:
    print("onnxruntime import error", e)
    ort = None

try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
except Exception:
    DeepSort = None


# ============================================================
#   SSD-Mobilenet postprocess
# ============================================================

def ssd_postprocess(boxes, classes, scores, num_det, conf_thr, img_w, img_h):
    """
    boxes: [num_det, 4] normalized (ymin, xmin, ymax, xmax)
    classes: [num_det]
    scores: [num_det]
    num_det: float -> convert to int

    Return: list of (x,y,w,h,score)
    """
    results = []
    count = int(num_det)

    for i in range(count):
        cls = int(classes[i])
        score = float(scores[i])
        if score < conf_thr:
            continue
        if cls != 1:  # COCO: person=1
            continue

        ymin, xmin, ymax, xmax = boxes[i]

        x = xmin * img_w
        y = ymin * img_h
        w = (xmax - xmin) * img_w
        h = (ymax - ymin) * img_h

        results.append((x, y, w, h, score))

    return results


# ============================================================
# PeopleDetector Node
# ============================================================

class PeopleDetector(Node):
    def __init__(self):
        super().__init__("people_detector")

        self.bridge = CvBridge()
        self.session = None

        # ---------------- Parameters ----------------
        self.declare_parameters("", [
            ("image_topic", "/camera/image_raw"),
            ("camera_info_topic", "/camera/camera_info"),
            ("det_topic", "/detections/person"),
            ("model_path", ""),
            ("conf_threshold", 0.4),
            ("infer_every_n", 1),

            # tracking
            ("use_tracker", True),
            ("tracker_max_age", 25),
            ("tracker_n_init", 3),
            ("tracker_max_cosine_distance", 0.2),
            ("tracker_nn_budget", 100),
        ])

        (
            self.image_topic,
            info_topic,
            self.det_topic,
            self.model_path,
            self.conf_thr,
            self.infer_every_n,
            self.use_tracker,
            self.trk_max_age,
            self.trk_n_init,
            self.trk_cos_thr,
            self.trk_nn_budget,
        ) = [p.value for p in self.get_parameters([
            "image_topic", "camera_info_topic", "det_topic",
            "model_path", "conf_threshold", "infer_every_n",
            "use_tracker", "tracker_max_age", "tracker_n_init",
            "tracker_max_cosine_distance", "tracker_nn_budget"
        ])]

        # debug output
        self.debug_topic = "/camera/people_debug"
        self.pub_dbg = self.create_publisher(Image, self.debug_topic, 10)

        # publishers
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)

        # subscribers
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self.last_info = None
        self.camera_frame_id = "camera"
        self.frame_idx = 0

        # ---------------- Load Model ----------------
        if self.model_path and os.path.exists(self.model_path):
            so = ort.SessionOptions()
            so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            try:
                self.session = ort.InferenceSession(
                    self.model_path, sess_options=so,
                    providers=["CPUExecutionProvider"]
                )
                self.input_name = self.session.get_inputs()[0].name
                self.get_logger().info(f"Loaded SSD model: {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"ONNX load failed: {e}")
        else:
            self.get_logger().error(f"Model path invalid: {self.model_path}")

        # ---------------- Tracker ----------------
        self.tracker = None
        if self.use_tracker:
            if DeepSort is None:
                self.get_logger().warn("DeepSORT not installed.")
            else:
                self.tracker = DeepSort(
                    max_age=int(self.trk_max_age),
                    n_init=int(self.trk_n_init),
                    max_cosine_distance=float(self.trk_cos_thr),
                    nn_budget=int(self.trk_nn_budget),
                    embedder="mobilenet",
                    half=True,
                    bgr=True,
                )
                self.get_logger().info("DeepSORT tracker initialized.")

    # ---------------------------------------------------------
    def on_info(self, msg: CameraInfo):
        self.last_info = msg
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id

    # ---------------------------------------------------------
    def on_image(self, img_msg: Image):
        self.frame_idx += 1

        dets_msg = Detection2DArray()
        dets_msg.header = img_msg.header

        if self.session is None or self.frame_idx % int(self.infer_every_n) != 0:
            self.pub_det.publish(dets_msg)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            self.pub_det.publish(dets_msg)
            return

        h, w = img.shape[:2]

        # ---------------- Preprocess for SSD ----------------
        inp = cv2.resize(img, (300, 300))
        inp = inp[:, :, ::-1]  # BGR → RGB
        inp = inp.astype(np.uint8)
        inp = inp[None, ...]  # (1,300,300,3)

        # ---------------- Inference ----------------
        try:
            t0 = time.time()
            outputs = self.session.run(None, {self.input_name: inp})
            t1 = time.time()
            #self.get_logger().info(f"SSD inference: {(t1 - t0)*1000:.1f} ms")
        except Exception as e:
            self.get_logger().error(f"SSD inference failed: {e}")
            self.pub_det.publish(dets_msg)
            return

        boxes = outputs[0][0]        # [num_det, 4]
        classes = outputs[1][0]      # [num_det]
        scores = outputs[2][0]       # [num_det]
        num_det = outputs[3][0]      # float

        detections = ssd_postprocess(
            boxes, classes, scores, num_det,
            conf_thr=self.conf_thr,
            img_w=w, img_h=h
        )

        # ---------------- Tracking ----------------
        tracks = []
        if self.tracker and detections:
            ds = []
            for (x, y, bw, bh, sc) in detections:
                ds.append(([x, y, x + bw, y + bh], sc, 0))
            tracks = self.tracker.update_tracks(ds, frame=img)

        # ---------------- Build ROS Message ----------------
        if tracks:
            for tr in tracks:
                if not tr.is_confirmed():
                    continue
                x1, y1, x2, y2 = tr.to_ltrb()
                wbox = x2 - x1
                hbox = y2 - y1
                cx, cy = x1 + wbox/2, y1 + hbox/2

                d = Detection2D()
                d.header = img_msg.header
                d.bbox.center.position.x = cx
                d.bbox.center.position.y = cy
                d.bbox.size_x = wbox
                d.bbox.size_y = hbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f"person:{tr.track_id}"
                hyp.hypothesis.score = tr.det_conf or 1.0
                d.results.append(hyp)
                dets_msg.detections.append(d)
        else:
            for (x, y, bw, bh, sc) in detections:
                cx, cy = x + bw/2, y + bh/2
                d = Detection2D()
                d.header = img_msg.header
                d.bbox.center.position.x = cx
                d.bbox.center.position.y = cy
                d.bbox.size_x = bw
                d.bbox.size_y = bh
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score = sc
                d.results.append(hyp)
                dets_msg.detections.append(d)

        # ---------------- Debug Overlay ----------------
        dbg = img.copy()
        for d in dets_msg.detections:
            x = int(d.bbox.center.position.x - 0.5 * d.bbox.size_x)
            y = int(d.bbox.center.position.y - 0.5 * d.bbox.size_y)
            wbox = int(d.bbox.size_x)
            hbox = int(d.bbox.size_y)
            cv2.rectangle(dbg, (x, y), (x+wbox, y+hbox), (0,255,0), 2)
            if d.results:
                cv2.putText(dbg, d.results[0].hypothesis.class_id,
                            (x, max(0, y-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0,255,0), 1)

        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
        dbg_msg.header = img_msg.header
        self.pub_dbg.publish(dbg_msg)

        self.pub_det.publish(dets_msg)


def main():
    rclpy.init()
    rclpy.spin(PeopleDetector())
    rclpy.shutdown()
