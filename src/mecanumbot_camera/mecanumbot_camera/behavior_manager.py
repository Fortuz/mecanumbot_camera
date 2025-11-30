#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class BehaviorState:
    SEARCH = "SEARCH"
    TRACK_BALL = "TRACK_BALL"
    FETCH = "FETCH"
    FIND_OWNER = "FIND_OWNER"
    DELIVER = "DELIVER"


class BehaviorManager(Node):
    def __init__(self):
        super().__init__("behavior_manager")

        # ----------- PARAMETERS ------------
        self.image_width = 640
        self.fetch_threshold_px = 110  # when ball bbox.width >= this → close enough
        self.search_speed = 0.25
        self.Kp_rot = 0.0025
        self.Kp_fwd = 0.015
        self.max_ang = 0.8
        self.max_lin = 0.3
        self.ball_lost_timeout = 1.0  # seconds

        # Ball tracking state
        self.last_ball_time = 0.0
        self.ball_center_x = None
        self.ball_width_px = 0.0

        # Current FSM state
        self.state = BehaviorState.SEARCH

        # ----------- PUBLISHER ------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ----------- SUBSCRIBERS ----------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            Detection2DArray,
            '/detections/ball',
            self.ball_callback,
            qos
        )

        self.get_logger().info("Behavior Manager initialized.")

        # ----------- MAIN CONTROL LOOP -----------
        self.control_timer = self.create_timer(0.1, self.control_loop)


    # ==========================================
    # BALL DETECTION CALLBACK
    # ==========================================
    def ball_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        # Pick largest detection
        det = max(msg.detections, key=lambda d: d.bbox.size_x)

        cx = det.bbox.center.position.x
        w = det.bbox.size_x

        self.ball_center_x = cx
        self.ball_width_px = w
        self.last_ball_time = time.time()


    # ==========================================
    # MAIN FSM LOGIC
    # ==========================================
    def control_loop(self):
        now = time.time()
        ball_seen_recently = (now - self.last_ball_time) < self.ball_lost_timeout

        twist = Twist()

        # ========== STATE: SEARCH ================
        if self.state == BehaviorState.SEARCH:
            twist.angular.z = self.search_speed

            if ball_seen_recently:
                self.state = BehaviorState.TRACK_BALL
                self.get_logger().info("→ TRACK_BALL")

        # ========== STATE: TRACK_BALL ==============
        elif self.state == BehaviorState.TRACK_BALL:
            if not ball_seen_recently:
                self.state = BehaviorState.SEARCH
                self.get_logger().info("Ball lost → SEARCH")
            else:
                twist = self.compute_ball_tracking()

                # Check if close enough → FETCH
                if self.ball_width_px >= self.fetch_threshold_px:
                    self.state = BehaviorState.FETCH
                    self.get_logger().info("→ FETCH (ball close)")

        # ========== STATE: FETCH =================
        elif self.state == BehaviorState.FETCH:
            if not ball_seen_recently:
                self.state = BehaviorState.SEARCH
                self.get_logger().info("Ball lost during FETCH → SEARCH")
            else:
                twist = self.compute_ball_tracking(slow=True)

                if self.ball_width_px >= self.fetch_threshold_px:
                    twist = Twist()  # stop
                    self.cmd_pub.publish(twist)

                    self.get_logger().info("Ball fetched! → FIND_OWNER")
                    self.state = BehaviorState.FIND_OWNER

        # ========== STATE: FIND_OWNER (placeholder) ==========
        elif self.state == BehaviorState.FIND_OWNER:
            twist.angular.z = self.search_speed

            # owner recognition will come in next version
            # but now we stay rotating
            pass

        # ========== STATE: DELIVER (placeholder) =============
        elif self.state == BehaviorState.DELIVER:
            pass

        # --------- PUBLISH VELOCITY ---------
        self.cmd_pub.publish(twist)

    # ==========================================
    # BALL TRACKING CONTROL LAW
    # ==========================================
    def compute_ball_tracking(self, slow=False):
        twist = Twist()

        if self.ball_center_x is None:
            return twist

        center_x = self.image_width / 2.0
        error_x = self.ball_center_x - center_x

        # Angular control
        ang = -self.Kp_rot * error_x
        ang = max(min(ang, self.max_ang), -self.max_ang)

        # Forward control
        forward_gain = self.ball_width_px < self.fetch_threshold_px
        lin = 0.0
        if forward_gain:
            lin = self.Kp_fwd * (self.fetch_threshold_px - self.ball_width_px)
            lin = max(min(lin, self.max_lin), 0.0)

        if slow:
            lin *= 0.5
            ang *= 0.5

        twist.linear.x = lin
        twist.angular.z = ang

        return twist


def main():
    rclpy.init()
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
