#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
try:
    from mecanumbot_msgs.msg import AccessMotorCmd, OpenCRState
except ImportError:
    AccessMotorCmd = None
    OpenCRState = None


class BehaviorState:
    SEARCH = "SEARCH"
    TRACK_BALL = "TRACK_BALL"
    FETCH = "FETCH"
    GRASP = "GRASP"
    FIND_OWNER = "FIND_OWNER"
    DELIVER = "DELIVER"


class BehaviorManager(Node):
    BEHAVIOR_PARAMETER_DEFAULTS = {
        "image_width": 640,
        "fetch_enter_px": 200,
        "fetch_stop_px": 280,
        "REQUIRED_BALL_STABLE": 3,
        "REQUIRED_PERSON_STABLE": 3,
        "ball_lost_timeout": 1.0,
        "person_lost_timeout": 1.0,
        "Kp_rot": 0.0025,
        "Kp_fwd": 0.015,
        "max_ang": 0.8,
        "max_lin": 0.30,
        "search_speed": 0.25,
        "owner_threshold_px": 150,
        "grasp_duration": 1.0,
        "min_find_owner_time": 1.0,
        "new_ball_absent_time": 2.0,
    }
    INTEGER_BEHAVIOR_PARAMETERS = {
        "image_width",
        "fetch_enter_px",
        "fetch_stop_px",
        "REQUIRED_BALL_STABLE",
        "REQUIRED_PERSON_STABLE",
        "owner_threshold_px",
    }
    NONNEGATIVE_BEHAVIOR_PARAMETERS = {
        "ball_lost_timeout",
        "person_lost_timeout",
        "grasp_duration",
        "min_find_owner_time",
        "new_ball_absent_time",
        "Kp_rot",
        "Kp_fwd",
        "max_ang",
        "max_lin",
    }

    def __init__(self):
        super().__init__("behavior_manager")

        # ------- BEHAVIOR PARAMETERS -------
        self.declare_parameters("", list(self.BEHAVIOR_PARAMETER_DEFAULTS.items()))
        self._load_behavior_parameters()

        self.twist = None

        # Ball state
        self.last_ball_time = -1e9
        self.ball_center_x = None
        self.ball_width_px = 0.0
        self.ball_stable_frames = 0

        # Person state
        self.last_person_time = -1e9
        self.person_center_x = None
        self.person_width_px = 0.0
        self.person_stable_frames = 0

        # Grasp state
        self.grasp_start_time = None

        # Camera tilt control (mecanumbot accessory channel)
        self.declare_parameter("enable_camera_tilt_control", True)
        self.declare_parameter("camera_tilt_topic", "/cmd_accessory_pos")
        self.declare_parameter("camera_tilt_ball_n_pos", 5.3)    # slight down
        self.declare_parameter("camera_tilt_owner_n_pos", 8.2)   # slight up
        self.declare_parameter("camera_tilt_opencr_state_topic", "/opencr_state")
        self.declare_parameter("camera_tilt_use_opencr_gripper_passthrough", True)
        self.declare_parameter("camera_tilt_gripper_hold_left", 5.12)
        self.declare_parameter("camera_tilt_gripper_hold_right", 5.12)

        self.enable_camera_tilt_control = bool(self.get_parameter("enable_camera_tilt_control").value)
        self.camera_tilt_topic = self.get_parameter("camera_tilt_topic").value
        self.camera_tilt_ball_n_pos = float(self.get_parameter("camera_tilt_ball_n_pos").value)
        self.camera_tilt_owner_n_pos = float(self.get_parameter("camera_tilt_owner_n_pos").value)
        self.camera_tilt_opencr_state_topic = self.get_parameter("camera_tilt_opencr_state_topic").value
        self.camera_tilt_use_opencr_gripper_passthrough = bool(
            self.get_parameter("camera_tilt_use_opencr_gripper_passthrough").value
        )
        self.camera_tilt_gripper_hold_left = float(self.get_parameter("camera_tilt_gripper_hold_left").value)
        self.camera_tilt_gripper_hold_right = float(self.get_parameter("camera_tilt_gripper_hold_right").value)
        self.last_camera_tilt_target = None
        self.last_gripper_left = self.camera_tilt_gripper_hold_left
        self.last_gripper_right = self.camera_tilt_gripper_hold_right

        # FIND_OWNER timing
        self.find_owner_enter_time = None

        # Deliver state helper
        self.deliver_open_done = False
        self.deliver_start_time = None

        # Új labda várása delivery után
        self.waiting_for_new_ball = False
        self.last_delivery_time = None

        # FSM
        self.state = BehaviorState.SEARCH

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gripper_pub = self.create_publisher(Float64, "/cmd_accessory_pos", 10)
        self.accessory_pub = None
        if self.enable_camera_tilt_control:
            if AccessMotorCmd is None:
                self.get_logger().warn(
                    "Camera tilt control requested, but mecanumbot_msgs/AccessMotorCmd is unavailable. "
                    "Disabling camera tilt control."
                )
                self.enable_camera_tilt_control = False
            else:
                self.accessory_pub = self.create_publisher(AccessMotorCmd, self.camera_tilt_topic, 10)
                self.get_logger().info(
                    f"Camera tilt enabled on {self.camera_tilt_topic}: "
                    f"ball_n_pos={self.camera_tilt_ball_n_pos:.2f}, "
                    f"owner_n_pos={self.camera_tilt_owner_n_pos:.2f}"
                )
                if self.camera_tilt_use_opencr_gripper_passthrough:
                    if OpenCRState is None:
                        self.get_logger().warn(
                            "OpenCRState unavailable; using configured gripper hold fallback values for tilt messages."
                        )
                    else:
                        self.create_subscription(
                            OpenCRState,
                            self.camera_tilt_opencr_state_topic,
                            self.opencr_state_callback,
                            10,
                        )
                        self.get_logger().info(
                            f"Gripper passthrough enabled from {self.camera_tilt_opencr_state_topic}."
                        )

        # QoS
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscriptions
        self.create_subscription(
            Detection2DArray,
            "/detections/ball",
            self.ball_callback,
            qos
        )
        self.create_subscription(
            Detection2DArray,
            "/detections/person",
            self.person_callback,
            qos
        )

        self.get_logger().info("Behavior Manager initialized.")
        self.update_camera_tilt_target()
        self.add_on_set_parameters_callback(self._on_behavior_param_change)

        # Timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

    # ==========================================================
    def _load_behavior_parameters(self):
        values = {}
        for name in self.BEHAVIOR_PARAMETER_DEFAULTS:
            value = self.get_parameter(name).value
            converted, reason = self._coerce_behavior_parameter(name, value)
            if reason:
                raise ValueError(reason)
            values[name] = converted

        reason = self._validate_behavior_parameters(values)
        if reason:
            raise ValueError(reason)

        for name, value in values.items():
            setattr(self, name, value)

    def _coerce_behavior_parameter(self, name, value):
        if name in self.INTEGER_BEHAVIOR_PARAMETERS:
            if isinstance(value, bool) or not isinstance(value, int):
                return None, f"{name} must be an integer"
            return int(value), None

        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, f"{name} must be numeric"
        return float(value), None

    def _validate_behavior_parameters(self, values):
        if values["image_width"] <= 0:
            return "image_width must be > 0"
        if values["fetch_enter_px"] <= 0:
            return "fetch_enter_px must be > 0"
        if values["fetch_stop_px"] <= 0:
            return "fetch_stop_px must be > 0"
        if values["fetch_enter_px"] >= values["fetch_stop_px"]:
            return "fetch_enter_px must be < fetch_stop_px"
        if values["owner_threshold_px"] <= 0:
            return "owner_threshold_px must be > 0"

        for name in ("REQUIRED_BALL_STABLE", "REQUIRED_PERSON_STABLE"):
            if values[name] < 1:
                return f"{name} must be an integer >= 1"

        for name in self.NONNEGATIVE_BEHAVIOR_PARAMETERS:
            if values[name] < 0:
                return f"{name} must be >= 0"

        return None

    def _on_behavior_param_change(self, params):
        values = {
            name: getattr(self, name)
            for name in self.BEHAVIOR_PARAMETER_DEFAULTS
        }
        changes = {}

        for param in params:
            if param.name not in self.BEHAVIOR_PARAMETER_DEFAULTS:
                continue

            converted, reason = self._coerce_behavior_parameter(param.name, param.value)
            if reason:
                return SetParametersResult(successful=False, reason=reason)

            values[param.name] = converted
            changes[param.name] = converted

        reason = self._validate_behavior_parameters(values)
        if reason:
            return SetParametersResult(successful=False, reason=reason)

        for name, value in changes.items():
            old_value = getattr(self, name)
            setattr(self, name, value)
            if old_value != value:
                self.get_logger().info(f"[PARAM] {name} changed: {old_value} -> {value}")

        return SetParametersResult(successful=True)

    # ==========================================================
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ==========================================================
    # BALL CALLBACK
    # ==========================================================
    def ball_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return

        det = max(msg.detections, key=lambda d: d.bbox.size_x)
        self.ball_center_x = det.bbox.center.position.x
        self.ball_width_px = det.bbox.size_x
        self.last_ball_time = self.now()

    # ==========================================================
    # PERSON CALLBACK
    # ==========================================================
    def person_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return

        det = max(msg.detections, key=lambda d: d.bbox.size_x)

        self.person_center_x = det.bbox.center.position.x
        self.person_width_px = det.bbox.size_x
        self.last_person_time = self.now()


    # ==========================================================
    # GRIPPER HELPERS
    # ==========================================================
    def open_gripper(self):
        msg = Float64()
        msg.data = 1.0
        self.gripper_pub.publish(msg)
        self.get_logger().info("[GRIPPER] open (1.0)")

    def close_gripper(self):
        msg = Float64()
        msg.data = 0.0
        self.gripper_pub.publish(msg)
        self.get_logger().info("[GRIPPER] close (0.0)")

    # ==========================================================
    # CONTROL LOOP — MAIN FSM
    # ==========================================================
    def control_loop(self):
        now = self.now()

        ball_seen = (now - self.last_ball_time) < self.ball_lost_timeout
        person_seen = (now - self.last_person_time) < self.person_lost_timeout

        twist = Twist()

        # --- stabilitási számlálók frissítése ---
        if ball_seen:
            self.ball_stable_frames += 1
        else:
            self.ball_stable_frames = 0

        if person_seen:
            self.person_stable_frames += 1
        else:
            self.person_stable_frames = 0

        # ------------------------------------------------------
        # SEARCH
        # ------------------------------------------------------
        if self.state == BehaviorState.SEARCH:
            twist.angular.z = self.search_speed

            # ha delivery után vagyunk, ÚJ labdát várunk
            if self.waiting_for_new_ball:
                # csak akkor engedünk új kört, ha eltűnt a labda egy ideig
                if (not ball_seen) and self.last_delivery_time is not None \
                        and (now - self.last_delivery_time) > self.new_ball_absent_time:
                    self.waiting_for_new_ball = False
                    self.ball_stable_frames = 0  # új stabilitás mérés
                # amíg várunk, csak forog
            else:
                if ball_seen and self.ball_stable_frames >= self.REQUIRED_BALL_STABLE:
                    self.state = BehaviorState.TRACK_BALL
                    self.get_logger().info("→ TRACK_BALL")

        # ------------------------------------------------------
        # TRACK_BALL
        # ------------------------------------------------------
        elif self.state == BehaviorState.TRACK_BALL:
            if not ball_seen:
                self.get_logger().info("Ball lost → SEARCH")
                self.state = BehaviorState.SEARCH
            else:
                twist = self.compute_ball_control()

                if self.ball_width_px >= self.fetch_enter_px and \
                        self.ball_stable_frames >= self.REQUIRED_BALL_STABLE:
                    self.state = BehaviorState.FETCH
                    self.get_logger().info("→ FETCH")

        # ------------------------------------------------------
        # FETCH — finom közelítés
        # ------------------------------------------------------
        elif self.state == BehaviorState.FETCH:
            if not ball_seen:
                self.get_logger().info("Ball lost during FETCH → SEARCH")
                self.state = BehaviorState.SEARCH
            else:
                twist = self.compute_ball_control(slow=True)

                if self.ball_width_px >= self.fetch_stop_px and \
                        self.ball_stable_frames >= self.REQUIRED_BALL_STABLE:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info("Ball reached → GRASP")
                    self.state = BehaviorState.GRASP
                    self.start_grasp(now)

        # ------------------------------------------------------
        # GRASP — zárd a gripper-t, várj kicsit
        # ------------------------------------------------------
        elif self.state == BehaviorState.GRASP:
            twist = Twist()

            if self.grasp_start_time is None:
                self.start_grasp(now)
            elif (now - self.grasp_start_time) >= self.grasp_duration:
                self.get_logger().info("Grasp done → FIND_OWNER")
                self.state = BehaviorState.FIND_OWNER
                self.find_owner_enter_time = now
                # újraindítjuk a person stabilitást, hogy ne az előzmény döntsön
                self.person_stable_frames = 0
                self.last_person_time = -1e9

        # ------------------------------------------------------
        # FIND_OWNER — keresi az "ownert"
        # ------------------------------------------------------
        elif self.state == BehaviorState.FIND_OWNER:
            twist = Twist()

            # minimális időt töltünk FIND_OWNER-ben, mielőtt egyáltalán
            # engednénk a DELIVER-re váltást
            enough_time_spent = (now - self.find_owner_enter_time) >= self.min_find_owner_time

            if person_seen:
                # közelítsünk a személyhez
                twist = self.compute_person_control()

                if enough_time_spent \
                        and self.person_width_px >= self.owner_threshold_px \
                        and self.person_stable_frames >= self.REQUIRED_PERSON_STABLE:
                    self.get_logger().info("Owner reached → DELIVER")
                    self.state = BehaviorState.DELIVER
                    twist = Twist()  # megáll
                    self.deliver_open_done = False
                    self.deliver_start_time = now
            else:
                # körbeforog, hogy keressen személyt
                twist.angular.z = self.search_speed

        # ------------------------------------------------------
        # DELIVER
        # ------------------------------------------------------
        elif self.state == BehaviorState.DELIVER:
            twist = Twist()

            if not self.deliver_open_done:
                self.open_gripper()
                self.deliver_open_done = True
                self.last_delivery_time = now  # innen számoljuk, mikortól várunk új labdára

            # maradjon DELIVER-ben kicsit
            if (now - self.deliver_start_time) > 2.0:
                self.get_logger().info("Delivery complete → SEARCH")
                self.state = BehaviorState.SEARCH
                self.deliver_open_done = False
                # innentől új labdát várunk: a labdának el kell tűnnie egy időre
                self.waiting_for_new_ball = True
                self.ball_stable_frames = 0

        # publish
        if self.twist != twist:
            self.twist = twist
            self.update_camera_tilt_target()
            self.cmd_pub.publish(twist)

    def update_camera_tilt_target(self):
        if not self.enable_camera_tilt_control or self.accessory_pub is None:
            return

        if self.state in (BehaviorState.SEARCH, BehaviorState.TRACK_BALL, BehaviorState.FETCH, BehaviorState.GRASP):
            target = self.camera_tilt_ball_n_pos
        else:
            target = self.camera_tilt_owner_n_pos

        if self.last_camera_tilt_target is not None and abs(target - self.last_camera_tilt_target) < 1e-6:
            return

        cmd = AccessMotorCmd()
        cmd.n_pos = float(target)
        # AccessMotorCmd bundles neck + grippers, so keep gripper fields aligned with latest known values.
        cmd.gl_pos = float(self.last_gripper_left)
        cmd.gr_pos = float(self.last_gripper_right)
        self.accessory_pub.publish(cmd)
        self.last_camera_tilt_target = target
        self.get_logger().info(f"[CAM_TILT] n_pos={target:.2f} (state={self.state})")

    def opencr_state_callback(self, msg):
        self.last_gripper_left = float(msg.pos_gl) / 100.0
        self.last_gripper_right = float(msg.pos_gr) / 100.0

    # ==========================================================
    def start_grasp(self, now: float):
        self.close_gripper()
        self.grasp_start_time = now

    # ==========================================================
    # CONTROL HELPERS
    # ==========================================================
    def compute_ball_control(self, slow: bool = False) -> Twist:
        twist = Twist()
        if self.ball_center_x is None:
            return twist

        center = self.image_width / 2
        err = self.ball_center_x - center

        ang = -self.Kp_rot * err
        ang = max(min(ang, self.max_ang), -self.max_ang)

        lin = 0.0
        if self.ball_width_px < self.fetch_stop_px:
            lin = self.Kp_fwd * (self.fetch_stop_px - self.ball_width_px)
            lin = max(min(lin, self.max_lin), 0.0)

        if slow:
            ang *= 0.5
            lin *= 0.5

        twist.angular.z = ang
        twist.linear.x = lin
        return twist

    def compute_person_control(self) -> Twist:
        twist = Twist()
        if self.person_center_x is None:
            return twist

        center = self.image_width / 2
        err = self.person_center_x - center

        ang = -self.Kp_rot * err
        ang = max(min(ang, self.max_ang), -self.max_ang)

        lin = 0.0
        if self.person_width_px < self.owner_threshold_px:
            lin = self.Kp_fwd * (self.owner_threshold_px - self.person_width_px)
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
