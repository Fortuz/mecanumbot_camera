#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class PeopleDetectorOptimized(Node):
    def __init__(self):
        super().__init__('people_detector')

        # ---- OpenCV optimalizációk ARM-on ----
        cv2.setUseOptimized(True)
        cv2.setNumThreads(1)

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Kamera subscriber
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            qos
        )

        # Detekció publikáció
        self.pub = self.create_publisher(
            Detection2DArray,
            '/people_detections',
            qos
        )

        # HOG emberdetektor inicializálása
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetectorOptimized())

        # csak minden N-edik frame
        self.frame_skip = 3
        self.frame_count = 0

        self.get_logger().info("Optimized Pi 3 PeopleDetectorOptimized node started.")

    def callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return  # skipping frames to save CPU

        # image → cv2
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {str(e)}")
            return

        # ↓ Pi 3 számára kritikus gyorsítás ↓
        # alacsony felbontás
        small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)

        # HOG emberek
        (rects, _) = self.hog.detectMultiScale(
            small,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05
        )

        # Detekciók összeállítása
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for (x, y, w, h) in rects:
            det = Detection2D()
            det.bbox.center.position.x = float(x + w / 2)
            det.bbox.center.position.y = float(y + h / 2)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"
            hyp.hypothesis.score = 0.7
            det.results.append(hyp)

            detections_msg.detections.append(det)

        self.pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PeopleDetectorOptimized()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
