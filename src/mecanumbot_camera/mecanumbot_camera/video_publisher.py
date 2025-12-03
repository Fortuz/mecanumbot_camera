#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")

        # Fixed camera input – the mjpeg stream produced by rpicam-vid
        self.stream_path = "/tmp/cam.mjpg"

        self.get_logger().info(f"Opening MJPEG stream: {self.stream_path}")

        self.cap = cv2.VideoCapture(self.stream_path)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open MJPEG camera stream!")

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        # 30 FPS stream
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn("No frame received from camera stream.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.width = frame.shape[1]
        info.height = frame.shape[0]
        self.pub_info.publish(info)

def main():
    rclpy.init()
    rclpy.spin(VideoPublisher())
    rclpy.shutdown()
