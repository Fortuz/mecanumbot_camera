#!/usr/bin/env python3
import os
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from tflite_runtime.interpreter import Interpreter


PERSON_CLASS_ID = 1  # COCO person

def nms(boxes, scores, iou_threshold=0.5):
    indices = []
    if len(boxes) == 0:
        return []

    boxes = np.array(boxes)
    scores = np.array(scores)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]  # sort by score desc

    while order.size > 0:
        i = order[0]
        indices.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        union = areas[i] + areas[order[1:]] - inter

        iou = inter / union
        keep = np.where(iou < iou_threshold)[0]

        order = order[keep + 1]

    return indices


class PeopleDetectorTFLite(Node):
    def __init__(self):
        super().__init__("people_detector_tflite")

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("det_topic", "/detections/person")
        self.declare_parameter("model_path", "ssd_mobilenet_v1.tflite")
        #self.declare_parameter("conf_threshold", 0.4)
        self.declare_parameter("conf_threshold", 0.85)
        self.declare_parameter("infer_every_n", 1)

        self.image_topic = self.get_parameter("image_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.det_topic = self.get_parameter("det_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.conf_thr = float(self.get_parameter("conf_threshold").value)
        self.infer_every_n = int(self.get_parameter("infer_every_n").value)

        # Debug topic
        self.pub_dbg = self.create_publisher(Image, "/camera/people_debug", 10)

        # Detection output
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)

        # Subscriptions
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self.last_info = None
        self.frame_idx = 0

        # ---------------- Load TFLite model ----------------
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"TFLite model not found: {self.model_path}")
            return

        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        input_details = self.interpreter.get_input_details()
        output_details = self.interpreter.get_output_details()

        # SSD Mobilenet V1 TFLite input/output spec
        self.input_index = input_details[0]["index"]
        self.input_height = input_details[0]["shape"][1]
        self.input_width = input_details[0]["shape"][2]

        self.output_boxes = output_details[0]["index"]
        self.output_classes = output_details[1]["index"]
        self.output_scores = output_details[2]["index"]
        self.output_num = output_details[3]["index"]

        self.get_logger().info(f"Loaded TFLite SSD model: {self.model_path}")
        input_details = self.interpreter.get_input_details()
        self.get_logger().info(f"INPUT: {input_details}")

    # ---------------------------------------------------------
    def on_info(self, msg: CameraInfo):
        self.last_info = msg

    # ---------------------------------------------------------
    def on_image(self, img_msg: Image):
        self.frame_idx += 1

        dets_msg = Detection2DArray()
        dets_msg.header = img_msg.header

        if self.frame_idx % self.infer_every_n != 0:
            self.pub_det.publish(dets_msg)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except:
            self.pub_det.publish(dets_msg)
            return

        h, w = img.shape[:2]

        # -------- Preprocess --------
        resized = cv2.resize(img, (self.input_width, self.input_height))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(rgb, axis=0).astype(np.uint8)

        # -------- Inference --------
        t0 = time.time()
        self.interpreter.set_tensor(self.input_index, inp)
        self.interpreter.invoke()
        t1 = time.time()

        #self.get_logger().info(f"TFLite inference: {(t1-t0)*1000:.1f} ms")

        #boxes = self.interpreter.get_tensor(self.output_boxes)[0]
        #classes = self.interpreter.get_tensor(self.output_classes)[0]
        #scores = self.interpreter.get_tensor(self.output_scores)[0]
        #num = int(self.interpreter.get_tensor(self.output_num)[0])
        boxes   = self.interpreter.get_tensor(167)[0]   # (10,4)
        classes = self.interpreter.get_tensor(168)[0]   # (10,)
        scores  = self.interpreter.get_tensor(169)[0]   # (10,)
        num     = int(self.interpreter.get_tensor(170)[0])


        # -------- Postprocess --------

        has_person = 0 in classes[:num]
        if has_person == False:
            self.pub_det.publish(dets_msg)
            pass

        detections = []

        for i in range(num):
            cls = int(classes[i])
            score = float(scores[i])

            if score < self.conf_thr:
                continue

            if cls != 0:
                continue

            ymin, xmin, ymax, xmax = boxes[i]

            x = xmin * w
            y = ymin * h
            bw = (xmax - xmin) * w
            bh = (ymax - ymin) * h

            detections.append((x, y, bw, bh, score))

        # -------- Publish detections (always AFTER NMS) --------
        for (x, y, bw, bh, sc) in detections:
            d = Detection2D()
            d.header = img_msg.header
            d.bbox.center.position.x = x + bw / 2
            d.bbox.center.position.y = y + bh / 2
            d.bbox.size_x = bw
            d.bbox.size_y = bh

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"
            hyp.hypothesis.score = sc
            d.results.append(hyp)

            dets_msg.detections.append(d)

        # -------- Debug overlay --------
        dbg = img.copy()
        for (x, y, bw, bh, sc) in detections:
            x1 = int(x)
            y1 = int(y)
            x2 = int(x + bw)
            y2 = int(y + bh)
            cv2.rectangle(dbg, (x1, y1), (x2, y2), (0,255,0), 2)

        self.get_logger().info(f"CLASSES: {classes[:num]}")
        self.get_logger().info(f"SCORES: {scores[:num]}")

        self.pub_det.publish(dets_msg)
        self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))


def main():
    rclpy.init()
    node = PeopleDetectorTFLite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
