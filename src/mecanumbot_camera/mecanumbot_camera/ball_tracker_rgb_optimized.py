#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

class BallTrackerRGBOptimized(Node):
    def __init__(self):
        super().__init__("ball_tracker_rgb")

        # OpenCV optimalizációk ARM-on
        cv2.setUseOptimized(True)
        cv2.setNumThreads(1)

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.callback,
            qos
        )

        self.pub = self.create_publisher(
            Detection2DArray,
            "/ball_detections",
            qos
        )

        # Paraméterek (állíthatóak params.yaml-ból)
        self.declare_parameter("h_low", 20)
        self.declare_parameter("h_high", 35)
        self.declare_parameter("s_low", 80)
        self.declare_parameter("s_high", 255)
        self.declare_parameter("v_low", 80)
        self.declare_parameter("v_high", 255)

        self.declare_parameter("frame_skip", 2)
        self.frame_skip = self.get_parameter("frame_skip").value
        self.frame_count = 0

        self.get_logger().info("Pi 3 optimized BallTracker RGB started")

    def callback(self, msg):
        # Frame skipping → CPU spórolás
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {str(e)}")
            return

        # Kép kisebbre → extrém gyorsítás
        small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_LINEAR)

        # HSV konverzió
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        # Paraméterek betöltése
        h_low = self.get_parameter("h_low").value
        h_high = self.get_parameter("h_high").value
        s_low = self.get_parameter("s_low").value
        s_high = self.get_parameter("s_high").value
        v_low = self.get_parameter("v_low").value
        v_high = self.get_parameter("v_high").value

        # Maszk
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)

        # Noise csökkentés (PC-n több lenne, de Pi3-on minimalizáljuk)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        # Kontúrok keresése
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 50:  # Pi3 optimal threshold
                x, y, w, h = cv2.boundingRect(c)

                det = Detection2D()
                det.bbox.center.position.x = float(x + w / 2)
                det.bbox.center.position.y = float(y + h / 2)
                det.bbox.size_x = float(w)
                det.bbox.size_y = float(h)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "ball"
                hyp.hypothesis.score = 0.9
                det.results.append(hyp)

                detections_msg.detections.append(det)

        self.pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BallTrackerRGBOptimized()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
