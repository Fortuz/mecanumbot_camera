#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge

class OverlayFused(Node):
    def __init__(self):
        super().__init__('overlay_fused')

        # params
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('person_det_topic', '/detections/person')
        self.declare_parameter('ball_det_topic',   '/detections/ball')
        self.declare_parameter('out_topic',        '/camera/fused_debug')

        self.image_topic  = self.get_parameter('image_topic').value
        self.person_topic = self.get_parameter('person_det_topic').value
        self.ball_topic   = self.get_parameter('ball_det_topic').value
        self.out_topic    = self.get_parameter('out_topic').value

        self.bridge = CvBridge()
        self.last_person = None
        self.last_ball   = None

        # subs
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, self.person_topic, self.on_person, 10)
        self.create_subscription(Detection2DArray, self.ball_topic,   self.on_ball,   10)

        # pub
        self.pub = self.create_publisher(Image, self.out_topic, 10)

        self.get_logger().info(f"OverlayFused listening on "
                               f"{self.image_topic}, {self.person_topic}, {self.ball_topic} -> {self.out_topic}")

    def on_person(self, msg: Detection2DArray): self.last_person = msg
    def on_ball(self,   msg: Detection2DArray): self.last_ball   = msg

    def on_image(self, img_msg: Image):
        # convert
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        dbg = frame.copy()
        stamp = img_msg.header.stamp
        frame_id = img_msg.header.frame_id or 'camera'

        # Draw helpers
        def draw_det(d: Detection2D, color, label_prefix=''):
            x = int(d.bbox.center.position.x - 0.5*d.bbox.size_x)
            y = int(d.bbox.center.position.y - 0.5*d.bbox.size_y)
            w = int(d.bbox.size_x); h = int(d.bbox.size_y)
            cv2.rectangle(dbg, (x,y), (x+w, y+h), color, 2)
            if d.results:
                cls = d.results[0].hypothesis.class_id or ''
                sc  = d.results[0].hypothesis.score
                label = f"{label_prefix}{cls} {sc:.2f}" if cls else f"{label_prefix}{sc:.2f}"
                cv2.putText(dbg, label, (x, max(0,y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # People (green)
        if self.last_person and self.last_person.detections:
            for d in self.last_person.detections:
                draw_det(d, (0,255,0), '')  # green

        # Ball (magenta)
        if self.last_ball and self.last_ball.detections:
            for d in self.last_ball.detections:
                draw_det(d, (255,0,255), '')  # magenta

        out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(OverlayFused())
    rclpy.shutdown()
