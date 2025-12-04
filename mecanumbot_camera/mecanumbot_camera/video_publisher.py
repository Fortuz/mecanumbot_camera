#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .camera_picam_reader import PiCamReader
import time

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cam = PiCamReader(width=640, height=480, fps=30)

        self.timer = self.create_timer(1/30.0, self.tick)

    def tick(self):
        frame = self.cam.read()
        if frame is None:
            self.get_logger().warn("No frame received yet.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = VideoPublisher()
    rclpy.spin(node)
    node.cam.stop()
    rclpy.shutdown()
