#!/usr/bin/env python3
import rclpy, cv2, threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

class DetectionOverlay(Node):
    def __init__(self):
        super().__init__('detection_overlay')
        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('dets_topic',  '/detections/person'),
            ('out_image_topic', '/image_people_debug'),
            ('label', 'person')
        ])
        p = [x.value for x in self.get_parameters(['image_topic','dets_topic','out_image_topic','label'])]
        self.image_topic, self.dets_topic, self.out_topic, self.label = p

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.out_topic, 10)

        self._lock = threading.Lock()
        self._last_dets = None  # type: Detection2DArray

        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(Detection2DArray, self.dets_topic, self.on_dets, 10)
        self.get_logger().info(f"Overlay listening image={self.image_topic} dets={self.dets_topic} -> out={self.out_topic}")

    def on_dets(self, dets: Detection2DArray):
        with self._lock:
            self._last_dets = dets

    def on_image(self, img_msg: Image):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        with self._lock:
            dets = self._last_dets

        if dets and dets.detections:
            for d in dets.detections:
                # Detection2D bbox is center (x,y) + size (w,h)
                cx = d.bbox.center.position.x
                cy = d.bbox.center.position.y
                w  = d.bbox.size_x
                h  = d.bbox.size_y
                x1 = int(cx - w/2); y1 = int(cy - h/2)
                x2 = int(cx + w/2); y2 = int(cy + h/2)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)

                score = None
                if d.results:
                    score = getattr(d.results[0].hypothesis, 'score', None)
                label = self.label if not d.results else d.results[0].hypothesis.class_id or self.label
                txt = f"{label}" + (f" {score:.2f}" if score is not None else "")
                cv2.putText(img, txt, (x1, max(0, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)

        self.pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))

def main():
    rclpy.init()
    rclpy.spin(DetectionOverlay())
    rclpy.shutdown()
