#!/usr/bin/env python3
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # ----------- CAMERA SOURCE MODIFIED HERE -----------
        # Read Pi Camera stream from rpicam-vid MJPEG FIFO
        fifo_path = "/tmp/cam.mjpg"
        self.cap = cv2.VideoCapture(fifo_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open {fifo_path}. "
                                    f"Make sure rpicam-vid is running:")
            self.get_logger().error(
                "    rpicam-vid --width 640 --height 480 --codec mjpeg "
                "--inline -o /tmp/cam.mjpg"
            )
        # ----------------------------------------------------

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.image_topic, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        # Fixed publish rate (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        ok, frame = self.cap.read()

        if not ok or frame is None:
            self.get_logger().warn("No frame received from camera stream.")
            return

        # Convert OpenCV frame → ROS2 Image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        # Publish CameraInfo with matching timestamp
        info = CameraInfo()
        info.header = msg.header
        info.width = frame.shape[1]
        info.height = frame.shape[0]
        self.pub_info.publish(info)

def main():
    rclpy.init()
    node = VideoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
