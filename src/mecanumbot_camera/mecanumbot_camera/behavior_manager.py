#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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

        # -------- PARAMETERS --------
        self.image_width = 640
        self.fetch_threshold_px = 110
        self.person_close_threshold_px = 130

        # Speed settings
        self.search_speed = 0.25
        self.Kp_rot = 0.0025
        self.Kp_fwd = 0.015
        self.max_ang = 0.8
        self.max_lin = 0.3
        self.ball_lost_timeout = 1.0
        self.person_lost_timeout = 1.0

        # Detection memory
        self.last_ball_time = 0.0
        self.ball_center_x = None
        self.ball_width_px = 0.0

        self.last_person_time = 0.0
        self.person_center_x = None
        self.person_width_px = 0.0

        self.state = BehaviorState.SEARCH

        # -------- PUBLISHER ----------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # -------- SUBSCRIBERS --------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Ball detection
        self.create_subscription(
            Detection2DArray,
            "/detections/ball",
            self.ball_callback,
            qos
        )

        # Person detection
        self.create_subscription(
            Detection2DArray,
            "/detections/person",
            self.person_callback,
            qos
        )

        self.get_logger().info("Behavior Manager initialized.")

        # ----- CONTROL LOOP -----
        self.control_timer = self.create_timer(0.1, self.control_loop)

    # ============================================
    # BALL CALLBACK
    # ============================================
    def ball_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        det = max(msg.detections, key=lambda d: d.bbox.size_x)

        self.ball_center_x = det.bbox.center.position.x
        self.ball_width_px = det.bbox.size_x
        self.last_ball_time = time.time()

    # ============================================
    # PERSON CALLBACK
    # ============================================
    def person_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        det = max(msg.detections, key=lambda d: d.bbox.size_x)

        self.person_center_x = det.bbox.center.position.x
        self.person_width_px = det.bbox.size_x
        self.last_person_time = time.time()

    # ============================================
    # CONTROL LOOP (FSM)
    # ============================================
    def control_loop(self):
        now = time.time()
        ball_recent = (now - self.last_ball_time) < self.ball_lost_timeout
        person_recent = (now - self.last_person_time) < self.person_lost_timeout

        twist = Twist()

        # ---------------- SEARCH -----------------
        if self.state == BehaviorState.SEARCH:
            twist.angular.z = self.search_speed

            if ball_recent:
                self.state = BehaviorState.TRACK_BALL
                self.get_logger().info("→ TRACK_BALL")

        # ---------------- TRACK BALL -------------
        elif self.state == BehaviorState.TRACK_BALL:
            if not ball_recent:
                self.state = BehaviorState.SEARCH
                self.get_logger().info("Ball lost → SEARCH")
            else:
                twist = self.compute_ball_tracking()

                if self.ball_width_px >= self.fetch_threshold_px:
                    self.state = BehaviorState.FETCH
                    self.get_logger().info("→ FETCH")

        # ---------------- FETCH -------------------
        elif self.state == BehaviorState.FETCH:
            if not ball_recent:
                self.state = BehaviorState.SEARCH
                self.get_logger().info("Lost during FETCH → SEARCH")
            else:
                twist = self.compute_ball_tracking(slow=True)

                if self.ball_width_px >= self.fetch_threshold_px:
                    twist = Twist()  # Stop
                    self.cmd_pub.publish(twist)
                    self.get_logger().info("Ball fetched! → FIND_OWNER")
                    self.state = BehaviorState.FIND_OWNER

        # ---------------- FIND OWNER --------------
        elif self.state == BehaviorState.FIND_OWNER:
            twist.angular.z = self.search_speed

            if person_recent:
                self.state = BehaviorState.DELIVER
                self.get_logger().info("Owner found! → DELIVER")

        # ---------------- DELIVER -----------------
        elif self.state == BehaviorState.DELIVER:
            if not person_recent:
                self.get_logger().info("Owner lost → FIND_OWNER")
                self.state = BehaviorState.FIND_OWNER
            else:
                twist = self.compute_person_tracking()

                if self.person_width_px >= self.person_close_threshold_px:
                    twist = Twist()
                    self.get_logger().info("Reached owner! Stopping.")
        
        self.cmd_pub.publish(twist)

    # ============================================
    # BALL TRACKING CONTROL
    # ============================================
    def compute_ball_tracking(self, slow=False):
        if self.ball_center_x is None:
            return Twist()

        twist = Twist()
        center = self.image_width / 2
        error = self.ball_center_x - center

        ang = -self.Kp_rot * error
        ang = max(min(ang, self.max_ang), -self.max_ang)

        lin = 0.0
        if self.ball_width_px < self.fetch_threshold_px:
            lin = self.Kp_fwd * (self.fetch_threshold_px - self.ball_width_px)
            lin = max(min(lin, self.max_lin), 0.0)

        if slow:
            ang *= 0.5
            lin *= 0.5

        twist.angular.z = ang
        twist.linear.x = lin
        return twist

    # ============================================
    # PERSON TRACKING CONTROL
    # ============================================
    def compute_person_tracking(self):
        if self.person_center_x is None:
            return Twist()

        twist = Twist()
        center = self.image_width / 2
        error = self.person_center_x - center

        ang = -self.Kp_rot * error
        ang = max(min(ang, self.max_ang), -self.max_ang)

        lin = self.Kp_fwd * (self.person_close_threshold_px - self.person_width_px)
        lin = max(min(lin, self.max_lin), 0.0)

        twist.angular.z = ang
        twist.linear.x = lin
        return twist


def main():
    rclpy.init()
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
