#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import SetBool  # or your LED service type

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')

        # --- Subscriptions & Publishers ---
        self.sub_det = self.create_subscription(
            Detection2DArray,
            '/detections/ball',
            self.det_callback,
            10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.led_client = self.create_client(SetBool, '/mecanumbot_led')  # adjust if different name

        # --- Parameters ---
        self.image_width = 640.0
        self.center_x = self.image_width / 2.0
        self.target_width = 120.0
        self.Kp_rot = 0.002
        self.Kp_fwd = 0.005
        self.max_ang = 1.0
        self.max_lin = 0.4
        self.search_speed = 0.4
        self.threshold_px = 40
        self.lost_timeout = 1.0

        # --- Internal State ---
        self.ball_x = None
        self.ball_w = None
        self.last_seen = self.get_clock().now()
        self.led_state = None

        self.create_timer(0.1, self.control_loop)

    # -----------------------------
    def det_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        biggest = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        cx = getattr(biggest.bbox.center, "x", biggest.bbox.center.position.x)
        w = biggest.bbox.size_x

        self.ball_x = float(cx)
        self.ball_w = float(w)
        self.last_seen = self.get_clock().now()

    # -----------------------------
    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()
        time_since_seen = (now - self.last_seen).nanoseconds * 1e-9

        if self.ball_x is not None and time_since_seen < self.lost_timeout:
            # Ball visible
            error_x = self.ball_x - self.center_x
            error_w = self.target_width - (self.ball_w or self.target_width)

            if abs(error_x) < self.threshold_px:
                twist.angular.z = 0.0
            else:
                twist.angular.z = -self.Kp_rot * error_x
                twist.angular.z = max(-self.max_ang, min(self.max_ang, twist.angular.z))

            twist.linear.x = self.Kp_fwd * error_w
            twist.linear.x = max(0.0, min(self.max_lin, twist.linear.x))

            self.get_logger().info(
                f"Tracking ball: x={self.ball_x:.1f}, w={self.ball_w:.1f}, "
                f"ang={twist.angular.z:.3f}, lin={twist.linear.x:.3f}"
            )

            # LED feedback: green when tracking
            self.set_led(True)

        else:
            # Ball lost
            twist.angular.z = self.search_speed
            twist.linear.x = 0.0
            self.get_logger().info("Searching for ball...")
            # LED feedback: off or different color
            self.set_led(False)

        self.pub_cmd.publish(twist)

    # -----------------------------
    def set_led(self, state: bool):
        """Send LED on/off command if changed."""
        if self.led_client.wait_for_service(timeout_sec=0.1):
            if state != self.led_state:
                req = SetBool.Request()
                req.data = state
                self.led_client.call_async(req)
                self.led_state = state
        else:
            self.get_logger().warn("LED service not available")

# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
