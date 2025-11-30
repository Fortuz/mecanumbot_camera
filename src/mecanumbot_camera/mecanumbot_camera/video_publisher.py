#!/usr/bin/env python3
import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # ADD these params & attributes
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('frame_id', 'camera')

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.declare_parameter('video_path', '/ws/src/mecanumbot_camera/media/sample.mp4')
        self.video_path = self.get_parameter('video_path').value
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video {self.video_path}")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = fps if fps and fps > 0 else 30.0
        self.timer = self.create_timer(1.0/float(fps), self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().info("End of video; looping.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        # (optional but recommended) publish CameraInfo with the SAME header:
        info = CameraInfo()
        info.header = msg.header
        info.width = frame.shape[1]
        info.height = frame.shape[0]
        # (fill more intrinsics if you have them)
        self.pub_info.publish(info)

def main():
    rclpy.init()
    rclpy.spin(VideoPublisher())
    rclpy.shutdown()
