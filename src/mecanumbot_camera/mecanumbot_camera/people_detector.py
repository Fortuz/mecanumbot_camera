#!/usr/bin/env python3
import os
import rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, Point2, Color, TextAnnotation

try:
    import onnxruntime as ort
except Exception:
    ort = None

# Optional: DeepSORT (appearance-based tracking)
try:
    from deep_sort_realtime.deepsort_tracker import DeepSort
except Exception:
    DeepSort = None


def parse_yolo_outputs(outputs, num_classes=80, person_cls=0):
    """
    Accepts tensors shaped like:
      - YOLOv8: [N, 4+num_classes] = cx,cy,w,h + class scores
      - YOLOv5: [N, 5+num_classes] = cx,cy,w,h,objectness + class scores
    Returns:
      boxes_cxcywh (float32, Nx4), scores (float32, N)
    """
    if outputs.ndim == 3:
        if outputs.shape[0] == 1:
            outputs = outputs[0]
        else:
            if outputs.shape[2] >= 6:
                outputs = np.transpose(outputs, (0, 2, 1))[0]
            else:
                outputs = outputs[0]
    if outputs.ndim == 2 and outputs.shape[0] < outputs.shape[1] and outputs.shape[0] in (6, 84, 85):
        outputs = outputs.T

    if outputs.ndim != 2 or outputs.shape[1] < 6:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)

    D = outputs.shape[1]
    if D == 84:  # v8 (4 + 80)
        boxes = outputs[:, :4]
        class_scores = outputs[:, 4:4 + num_classes]
        scores = class_scores[:, person_cls]
    elif D == 85:  # v5 (4 + 1 + 80)
        boxes = outputs[:, :4]
        obj = outputs[:, 4]
        class_scores = outputs[:, 5:5 + num_classes]
        scores = class_scores[:, person_cls] * obj
    else:
        boxes = outputs[:, :4]
        class_scores = outputs[:, -num_classes:]
        scores = class_scores[:, person_cls]
    return boxes.astype(np.float32), scores.astype(np.float32)


class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')

        self.camera_frame_id = 'camera'
        self.last_info = None
        self.bridge = CvBridge()
        self.frame_idx = 0
        self.session = None
        self.input_name = None

        # -------- Params --------
        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('camera_info_topic', '/camera/camera_info'),
            ('det_topic', '/detections/person'),           # published detections/tracks
            ('model_path', ''),
            ('input_size', [640, 640]),
            ('conf_threshold', 0.30),
            ('iou_threshold', 0.45),
            ('infer_every_n', 1),
            ('person_class_ids', [0]),

            # NEW: tracking controls
            ('use_tracker', True),                         # turn tracking on/off
            ('tracker_max_age', 25),                       # frames to keep a lost track
            ('tracker_n_init', 3),                         # hits before confirming a track
            ('tracker_max_cosine_distance', 0.2),          # ReID distance threshold
            ('tracker_nn_budget', 100),                    # embedding cache size
        ])

        (
            self.image_topic,
            info_topic,
            self.det_topic,
            self.model_path,
            self.input_size,
            self.conf_thr,
            self.iou_thr,
            self.infer_every_n,
            self.person_ids,
            self.use_tracker,
            self.trk_max_age,
            self.trk_n_init,
            self.trk_cos_thr,
            self.trk_nn_budget,
        ) = [p.value for p in self.get_parameters([
            'image_topic', 'camera_info_topic', 'det_topic', 'model_path', 'input_size',
            'conf_threshold', 'iou_threshold', 'infer_every_n', 'person_class_ids',
            'use_tracker', 'tracker_max_age', 'tracker_n_init',
            'tracker_max_cosine_distance', 'tracker_nn_budget'
        ])]

        # Annotations + debug image
        self.ann_topic = self.declare_parameter('ann_topic', '/camera/annotations').get_parameter_value().string_value
        self.pub_ann = self.create_publisher(ImageAnnotations, self.ann_topic, 10)
        self.debug_topic = self.declare_parameter('people_debug_topic', '/camera/people_debug').get_parameter_value().string_value
        self.pub_dbg = self.create_publisher(Image, self.debug_topic, 10)

        # ROS I/O
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)

        # ---- ONNX init ----
        if ort is None:
            self.get_logger().warn("onnxruntime not installed; detector will publish empty arrays.")
        elif not self.model_path:
            self.get_logger().warn("No model_path provided; detector will publish empty arrays.")
        elif not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found: {self.model_path}; publishing empty arrays.")
        else:
            try:
                so = ort.SessionOptions()
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                so.intra_op_num_threads = 4
                so.inter_op_num_threads = 1
                self.session = ort.InferenceSession(
                    self.model_path,
                    sess_options=so,
                    providers=['CPUExecutionProvider']
                )
                self.input_name = self.session.get_inputs()[0].name
                self.get_logger().info(
                    f"Loaded ONNX model: {self.model_path} providers={self.session.get_providers()} input={self.input_name}"
                )
            except Exception as e:
                self.get_logger().error(f"ONNX load failed: {e}; publishing empty arrays.")
                self.session = None

        # ---- Tracker init ----
        self.tracker = None
        if self.use_tracker:
            if DeepSort is None:
                self.get_logger().warn("DeepSort not installed (pip install deep-sort-realtime). Running without tracking.")
            else:
                # appearance (ReID) + motion tracker
                self.tracker = DeepSort(
                    max_age=int(self.trk_max_age),
                    n_init=int(self.trk_n_init),
                    max_cosine_distance=float(self.trk_cos_thr),
                    nn_budget=int(self.trk_nn_budget),
                    embedder="mobilenet",              # lightweight default
                    half=True,                         # use fp16 when possible
                    bgr=True,                          # our frames are BGR
                )
                self.get_logger().info("DeepSort tracker initialised.")

    # ------------ Camera info ------------
    def on_info(self, msg: CameraInfo):
        self.last_info = msg
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id

    # ------------ Preprocess ------------
    def preprocess(self, img):
        h, w = img.shape[:2]
        inp_w, inp_h = int(self.input_size[0]), int(self.input_size[1])
        r = min(inp_w / w, inp_h / h)
        nw, nh = int(w * r), int(h * r)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((inp_h, inp_w, 3), dtype=np.uint8)
        canvas[:nh, :nw] = resized
        blob = canvas[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0  # BGR->RGB -> NCHW
        return blob, r

    # ------------ NMS ------------
    def nms(self, boxes_xywh, scores, iou_thr):
        b = [list(map(int, bb)) for bb in boxes_xywh]
        s = list(map(float, scores))
        idxs = cv2.dnn.NMSBoxes(b, s, score_threshold=float(self.conf_thr), nms_threshold=float(iou_thr))
        if len(idxs) == 0:
            return []
        return [int(i) for i in np.array(idxs).flatten()]

    # ------------ Main image callback ------------
    def on_image(self, img_msg: Image):
        self.frame_idx += 1

        stamp = img_msg.header.stamp
        if getattr(stamp, 'sec', 0) == 0 and getattr(stamp, 'nanosec', 0) == 0:
            stamp = self.get_clock().now().to_msg()
        frame_id = img_msg.header.frame_id or self.camera_frame_id

        dets_msg = Detection2DArray()
        dets_msg.header = img_msg.header

        # throttle or no model → publish empty (but keep cadence)
        if self.session is None or (self.frame_idx % max(1, int(self.infer_every_n)) != 0):
            self._publish_annotations_and_images(img=None, dets=dets_msg)
            self.pub_det.publish(dets_msg)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            self.pub_det.publish(dets_msg)
            return

        # --- Inference ---
        blob, r = self.preprocess(img)
        inp = blob[None, ...]
        try:
            outs = self.session.run(None, {self.input_name: inp})
        except Exception as e:
            self.get_logger().error(f"ORT inference failed: {e}")
            self.pub_det.publish(dets_msg)
            return

        pred = outs[0]
        boxes_cxcywh, person_scores = parse_yolo_outputs(
            pred,
            num_classes=80,
            person_cls=int(self.person_ids[0]) if len(self.person_ids) else 0
        )

        boxes_xywh, scores = [], []
        for (cx, cy, w, h), sc in zip(boxes_cxcywh, person_scores):
            sc = float(sc)
            if sc < float(self.conf_thr):
                continue
            x = (cx - w / 2.0) / r
            y = (cy - h / 2.0) / r
            bw = w / r
            bh = h / r
            if bw <= 1 or bh <= 1:
                continue
            boxes_xywh.append([x, y, bw, bh])
            scores.append(sc)

        keep = self.nms(boxes_xywh, scores, self.iou_thr) if boxes_xywh else []
        kept = [(boxes_xywh[i], scores[i]) for i in keep]

        # --- Optional tracking ---
        tracks = []
        if self.tracker is not None and kept:
            # DeepSORT expects a list of: [ [x1,y1,x2,y2], score, class ]
            ds_dets = []
            for (x, y, w, h), sc in kept:
                bbox = [x, y, x + w, y + h]
                ds_dets.append((bbox, float(sc), 0))  # class_id=0 (person)
            # Update with current frame (BGR)
            tracks = self.tracker.update_tracks(ds_dets, frame=img)

        # --- Build output message ---
        if tracks:
            # Use tracked outputs (stable IDs)
            for tr in tracks:
                if not tr.is_confirmed():
                    continue
                tid = tr.track_id
                x1, y1, x2, y2 = tr.to_ltrb()  # left, top, right, bottom
                w = x2 - x1
                h = y2 - y1
                cx, cy = x1 + w / 2.0, y1 + h / 2.0

                d = Detection2D()
                d.header.stamp = stamp
                d.header.frame_id = frame_id
                d.bbox.center.position.x = float(cx)
                d.bbox.center.position.y = float(cy)
                d.bbox.center.theta = 0.0
                d.bbox.size_x = float(w)
                d.bbox.size_y = float(h)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f"person:{tid}"  # encode track id
                hyp.hypothesis.score = float(tr.det_conf if tr.det_conf is not None else 1.0)
                d.results.append(hyp)
                dets_msg.detections.append(d)
        else:
            # Fall back to raw detections
            for (x, y, bw, bh), sc in kept:
                cx, cy = x + bw / 2.0, y + bh / 2.0
                d = Detection2D()
                d.header.stamp = stamp
                d.header.frame_id = frame_id
                d.bbox.center.position.x = float(cx)
                d.bbox.center.position.y = float(cy)
                d.bbox.center.theta = 0.0
                d.bbox.size_x = float(bw)
                d.bbox.size_y = float(bh)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score = float(sc)
                d.results.append(hyp)
                dets_msg.detections.append(d)

        # --- Debug overlay + annotations ---
        dbg = img.copy()
        color_box = (0, 255, 0)
        for d in dets_msg.detections:
            x = int(d.bbox.center.position.x - 0.5 * d.bbox.size_x)
            y = int(d.bbox.center.position.y - 0.5 * d.bbox.size_y)
            w = int(d.bbox.size_x)
            h = int(d.bbox.size_y)
            cv2.rectangle(dbg, (x, y), (x + w, y + h), color_box, 2)
            if d.results:
                label = d.results[0].hypothesis.class_id
                cv2.putText(dbg, label, (x, max(0, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_box, 1)

        dbg_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        dbg_msg.header.stamp = dets_msg.header.stamp
        dbg_msg.header.frame_id = dets_msg.header.frame_id
        self.pub_dbg.publish(dbg_msg)

        self._publish_annotations_and_images(img=dbg, dets=dets_msg)
        self.pub_det.publish(dets_msg)

    # ------------ Foxglove annotation helper ------------
    def _publish_annotations_and_images(self, img, dets: Detection2DArray):
        anns = ImageAnnotations()
        anns.points = []
        anns.texts = []
        for d in dets.detections:
            x = d.bbox.center.position.x - d.bbox.size_x * 0.5
            y = d.bbox.center.position.y - d.bbox.size_y * 0.5
            w = d.bbox.size_x
            h = d.bbox.size_y

            pa = PointsAnnotation()
            pa.timestamp = dets.header.stamp
            pa.type = PointsAnnotation.LINE_LOOP
            pa.points = [
                Point2(x=float(x),     y=float(y)),
                Point2(x=float(x+w),   y=float(y)),
                Point2(x=float(x+w),   y=float(y+h)),
                Point2(x=float(x),     y=float(y+h)),
            ]
            # Color expects floats, not ints
            pa.outline_color = Color(r=0.0, g=1.0, b=0.0, a=1.0)
            pa.thickness = 2.0
            anns.points.append(pa)


            if d.results:
                txt = TextAnnotation()
                txt.timestamp = dets.header.stamp
                txt.position = Point2(x=float(x), y=float(max(y-6, 0)))
                txt.text = d.results[0].hypothesis.class_id  # includes "person:<id>" when tracking
                anns.texts.append(txt)

        self.pub_ann.publish(anns)


def main():
    rclpy.init()
    rclpy.spin(PeopleDetector())
    rclpy.shutdown()
