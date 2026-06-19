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

MECANUMBOT_MIN_GRIPPER_POS = 1.6
MECANUMBOT_FRONT_GRIPPER_POS = 5.12
MECANUMBOT_MAX_GRIPPER_POS = 8.54

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
        "image_height": 480,
        "fetch_enter_px": 150,
        "fetch_stop_px": 240,
        "REQUIRED_BALL_STABLE": 3,
        "REQUIRED_PERSON_STABLE": 3,
        "ball_lost_timeout": 1.0,
        "person_lost_timeout": 1.0,
        "Kp_rot": 0.0025,
        "Kp_fwd": 0.015,
        "max_ang": 0.8,
        "max_lin": 0.30,
        "slow_ang_scale": 0.85,
        "slow_lin_scale": 0.50,
        "ball_close_turn_boost_start_px": 150.0,
        "ball_close_turn_boost": 1.8,
        "ball_forward_slowdown_start_px": 60.0,
        "ball_forward_stop_error_px": 180.0,
        "grasp_center_tolerance_px": 80.0,
        "search_speed": 0.125,
        "owner_threshold_px": 150,
        "grasp_duration": 1.0,
        "min_find_owner_time": 1.0,
        "new_ball_absent_time": 2.0,
    }
    INTEGER_BEHAVIOR_PARAMETERS = {
        "image_width",
        "image_height",
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
        "slow_ang_scale",
        "slow_lin_scale",
        "ball_close_turn_boost_start_px",
        "ball_close_turn_boost",
        "ball_forward_slowdown_start_px",
        "ball_forward_stop_error_px",
        "grasp_center_tolerance_px",
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
        self.ball_center_y = None
        self.ball_width_px = 0.0
        self.ball_height_px = 0.0
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
        self.declare_parameter("camera_tilt_ball_n_pos", 7.2)    # forward, slightly down
        self.declare_parameter("camera_tilt_owner_n_pos", 8.2)   # slight up
        self.declare_parameter("camera_tilt_search_sweep_enabled", True)
        self.declare_parameter("camera_tilt_search_min_n_pos", 5.8)
        self.declare_parameter("camera_tilt_search_max_n_pos", 7.4)
        self.declare_parameter("camera_tilt_search_sweep_period", 9.0)
        self.declare_parameter("camera_tilt_owner_search_sweep_enabled", True)
        self.declare_parameter("camera_tilt_owner_search_min_n_pos", 7.6)
        self.declare_parameter("camera_tilt_owner_search_max_n_pos", 8.4)
        self.declare_parameter("camera_tilt_owner_search_sweep_period", 7.0)
        self.declare_parameter("camera_tilt_publish_min_delta", 0.03)
        self.declare_parameter("camera_tilt_track_min_n_pos", 5.6)
        self.declare_parameter("camera_tilt_track_max_n_pos", 8.1)
        self.declare_parameter("camera_tilt_track_kp", 0.005)
        self.declare_parameter("camera_tilt_track_deadband_px", 24.0)
        self.declare_parameter("camera_tilt_track_close_down_n_pos", 5.7)
        self.declare_parameter("camera_tilt_track_close_start_px", 140.0)
        self.declare_parameter("camera_tilt_track_close_full_px", 260.0)
        self.declare_parameter("camera_tilt_opencr_state_topic", "/opencr_state")
        self.declare_parameter("camera_tilt_use_opencr_gripper_passthrough", True)
        self.declare_parameter("camera_tilt_gripper_hold_left", 5.12)
        self.declare_parameter("camera_tilt_gripper_hold_right", 5.12)

        self.enable_camera_tilt_control = bool(self.get_parameter("enable_camera_tilt_control").value)
        self.camera_tilt_topic = self.get_parameter("camera_tilt_topic").value
        self.camera_tilt_ball_n_pos = float(self.get_parameter("camera_tilt_ball_n_pos").value)
        self.camera_tilt_owner_n_pos = float(self.get_parameter("camera_tilt_owner_n_pos").value)
        self.camera_tilt_search_sweep_enabled = bool(
            self.get_parameter("camera_tilt_search_sweep_enabled").value
        )
        self.camera_tilt_search_min_n_pos = float(self.get_parameter("camera_tilt_search_min_n_pos").value)
        self.camera_tilt_search_max_n_pos = float(self.get_parameter("camera_tilt_search_max_n_pos").value)
        self.camera_tilt_search_sweep_period = float(self.get_parameter("camera_tilt_search_sweep_period").value)
        self.camera_tilt_owner_search_sweep_enabled = bool(
            self.get_parameter("camera_tilt_owner_search_sweep_enabled").value
        )
        self.camera_tilt_owner_search_min_n_pos = float(
            self.get_parameter("camera_tilt_owner_search_min_n_pos").value
        )
        self.camera_tilt_owner_search_max_n_pos = float(
            self.get_parameter("camera_tilt_owner_search_max_n_pos").value
        )
        self.camera_tilt_owner_search_sweep_period = float(
            self.get_parameter("camera_tilt_owner_search_sweep_period").value
        )
        self.camera_tilt_publish_min_delta = float(self.get_parameter("camera_tilt_publish_min_delta").value)
        self.camera_tilt_track_min_n_pos = float(self.get_parameter("camera_tilt_track_min_n_pos").value)
        self.camera_tilt_track_max_n_pos = float(self.get_parameter("camera_tilt_track_max_n_pos").value)
        self.camera_tilt_track_kp = float(self.get_parameter("camera_tilt_track_kp").value)
        self.camera_tilt_track_deadband_px = float(self.get_parameter("camera_tilt_track_deadband_px").value)
        self.camera_tilt_track_close_down_n_pos = float(
            self.get_parameter("camera_tilt_track_close_down_n_pos").value
        )
        self.camera_tilt_track_close_start_px = float(
            self.get_parameter("camera_tilt_track_close_start_px").value
        )
        self.camera_tilt_track_close_full_px = float(
            self.get_parameter("camera_tilt_track_close_full_px").value
        )
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
        self.gripper_pub = self.create_publisher(AccessMotorCmd, "/cmd_accessory_pos", 10)
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
                    f"owner_n_pos={self.camera_tilt_owner_n_pos:.2f}, "
                    f"search_sweep={self.camera_tilt_search_sweep_enabled} "
                    f"[{self.camera_tilt_search_min_n_pos:.2f}, {self.camera_tilt_search_max_n_pos:.2f}], "
                    f"owner_search_sweep={self.camera_tilt_owner_search_sweep_enabled} "
                    f"[{self.camera_tilt_owner_search_min_n_pos:.2f}, "
                    f"{self.camera_tilt_owner_search_max_n_pos:.2f}], "
                    f"track_range=[{self.camera_tilt_track_min_n_pos:.2f}, {self.camera_tilt_track_max_n_pos:.2f}]"
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
        if values["image_height"] <= 0:
            return "image_height must be > 0"
        if values["fetch_enter_px"] <= 0:
            return "fetch_enter_px must be > 0"
        if values["fetch_stop_px"] <= 0:
            return "fetch_stop_px must be > 0"
        if values["fetch_enter_px"] >= values["fetch_stop_px"]:
            return "fetch_enter_px must be < fetch_stop_px"
        if values["owner_threshold_px"] <= 0:
            return "owner_threshold_px must be > 0"
        if values["ball_forward_stop_error_px"] <= values["ball_forward_slowdown_start_px"]:
            return "ball_forward_stop_error_px must be > ball_forward_slowdown_start_px"

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
            self.get_logger().info("[ball_callback] no detections")
            return

        det = max(msg.detections, key=lambda d: d.bbox.size_x)
        self.ball_center_x = det.bbox.center.position.x
        self.ball_center_y = det.bbox.center.position.y
        self.ball_width_px = det.bbox.size_x
        self.ball_height_px = det.bbox.size_y
        self.last_ball_time = self.now()

    def format_ball_bbox_size(self):
        return f"{self.ball_width_px:.1f}x{self.ball_height_px:.1f}px"

    def ball_size_px(self):
        return max(self.ball_width_px, self.ball_height_px)

    def ball_is_centered_for_grasp(self):
        if self.ball_center_x is None:
            return False

        center = self.image_width / 2.0
        return abs(self.ball_center_x - center) <= self.grasp_center_tolerance_px

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
        msg = AccessMotorCmd()        
        msg.n_pos = self.last_camera_tilt_target
        # AccessMotorCmd bundles neck + grippers, so keep gripper fields aligned with latest known values.
        msg.gl_pos = MECANUMBOT_FRONT_GRIPPER_POS
        msg.gr_pos = MECANUMBOT_FRONT_GRIPPER_POS
        self.gripper_pub.publish(msg)
        self.get_logger().info("[GRIPPER] open (1.0)")
        self.last_gripper_left = msg.gl_pos
        self.last_gripper_right = msg.gr_pos

    def close_gripper(self):
        msg = AccessMotorCmd()       
        msg.n_pos = self.last_camera_tilt_target
        # AccessMotorCmd bundles neck + grippers, so keep gripper fields aligned with latest known values.
        msg.gl_pos = (MECANUMBOT_MAX_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
        msg.gr_pos = (MECANUMBOT_MIN_GRIPPER_POS + MECANUMBOT_FRONT_GRIPPER_POS)/2
        self.gripper_pub.publish(msg)
        self.get_logger().info("[GRIPPER] close (0.0)")
        self.last_gripper_left = msg.gl_pos
        self.last_gripper_right = msg.gr_pos

    # ==========================================================
    # CONTROL LOOP — MAIN FSM
    # ==========================================================
    def control_loop(self):
        self.get_logger().info("[control_loop]")

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
                    self.get_logger().info(f"→ TRACK_BALL ball_bbox={self.format_ball_bbox_size()}")

        # ------------------------------------------------------
        # TRACK_BALL
        # ------------------------------------------------------
        elif self.state == BehaviorState.TRACK_BALL:
            if not ball_seen:
                self.get_logger().info("Ball lost → SEARCH")
                self.state = BehaviorState.SEARCH
            else:
                twist = self.compute_ball_control(slow=True)

                if self.ball_size_px() >= self.fetch_enter_px and \
                        self.ball_stable_frames >= self.REQUIRED_BALL_STABLE:
                    self.state = BehaviorState.FETCH
                    self.get_logger().info(f"→ FETCH ball_bbox={self.format_ball_bbox_size()}")

        # ------------------------------------------------------
        # FETCH — finom közelítés
        # ------------------------------------------------------
        elif self.state == BehaviorState.FETCH:
            if not ball_seen:
                self.get_logger().info("Ball lost during FETCH → SEARCH")
                self.state = BehaviorState.SEARCH
            else:
                twist = self.compute_ball_control(slow=True)

                if self.ball_size_px() >= self.fetch_stop_px and \
                        self.ball_stable_frames >= self.REQUIRED_BALL_STABLE and \
                        self.ball_is_centered_for_grasp():
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

        self.update_camera_tilt_target(now)

        # publish
        if self.twist != twist:
            self.twist = twist
            self.cmd_pub.publish(twist)

    def compute_search_tilt_target(self, now: float) -> float:
        min_pos = min(self.camera_tilt_search_min_n_pos, self.camera_tilt_search_max_n_pos)
        max_pos = max(self.camera_tilt_search_min_n_pos, self.camera_tilt_search_max_n_pos)
        period = max(self.camera_tilt_search_sweep_period, 0.1)
        return self.compute_sweep_tilt_target(now, min_pos, max_pos, period)

    def compute_owner_search_tilt_target(self, now: float) -> float:
        min_pos = min(self.camera_tilt_owner_search_min_n_pos, self.camera_tilt_owner_search_max_n_pos)
        max_pos = max(self.camera_tilt_owner_search_min_n_pos, self.camera_tilt_owner_search_max_n_pos)
        period = max(self.camera_tilt_owner_search_sweep_period, 0.1)
        return self.compute_sweep_tilt_target(now, min_pos, max_pos, period)

    def compute_sweep_tilt_target(self, now: float, min_pos: float, max_pos: float, period: float) -> float:
        phase = (now % period) / period
        ratio = phase * 2.0 if phase < 0.5 else (1.0 - phase) * 2.0
        return min_pos + (max_pos - min_pos) * ratio

    def compute_track_ball_tilt_target(self) -> float:
        if self.ball_center_y is None:
            return self.camera_tilt_ball_n_pos

        image_center_y = self.image_height / 2.0
        err_y = self.ball_center_y - image_center_y
        if abs(err_y) < self.camera_tilt_track_deadband_px:
            err_y = 0.0

        target = self.camera_tilt_ball_n_pos - self.camera_tilt_track_kp * err_y

        close_start = min(self.camera_tilt_track_close_start_px, self.camera_tilt_track_close_full_px)
        close_full = max(self.camera_tilt_track_close_start_px, self.camera_tilt_track_close_full_px)
        close_span = max(close_full - close_start, 1.0)
        close_ratio = max(min((self.ball_size_px() - close_start) / close_span, 1.0), 0.0)
        close_target = self.camera_tilt_ball_n_pos + (
            self.camera_tilt_track_close_down_n_pos - self.camera_tilt_ball_n_pos
        ) * close_ratio
        target = min(target, close_target)

        min_pos = min(self.camera_tilt_track_min_n_pos, self.camera_tilt_track_max_n_pos)
        max_pos = max(self.camera_tilt_track_min_n_pos, self.camera_tilt_track_max_n_pos)
        return max(min(target, max_pos), min_pos)

    def update_camera_tilt_target(self, now: float = None):
        if not self.enable_camera_tilt_control or self.accessory_pub is None:
            return

        if self.state == BehaviorState.SEARCH and self.camera_tilt_search_sweep_enabled:
            if now is None:
                now = self.now()
            target = self.compute_search_tilt_target(now)
        elif self.state in (BehaviorState.TRACK_BALL, BehaviorState.FETCH):
            target = self.compute_track_ball_tilt_target()
        elif self.state == BehaviorState.FIND_OWNER and self.camera_tilt_owner_search_sweep_enabled:
            if now is None:
                now = self.now()
            target = self.compute_owner_search_tilt_target(now)
        elif self.state == BehaviorState.GRASP:
            target = self.camera_tilt_track_close_down_n_pos
        elif self.state in (BehaviorState.SEARCH, BehaviorState.TRACK_BALL, BehaviorState.FETCH):
            target = self.camera_tilt_ball_n_pos
        else:
            target = self.camera_tilt_owner_n_pos

        if self.last_camera_tilt_target is not None and \
                abs(target - self.last_camera_tilt_target) < self.camera_tilt_publish_min_delta:
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
        if self.ball_size_px() > self.ball_close_turn_boost_start_px:
            close_span = max(self.fetch_stop_px - self.ball_close_turn_boost_start_px, 1.0)
            close_ratio = max(min((self.ball_size_px() - self.ball_close_turn_boost_start_px) / close_span, 1.0), 0.0)
            ang *= 1.0 + close_ratio * self.ball_close_turn_boost
        ang = max(min(ang, self.max_ang), -self.max_ang)

        lin = 0.0
        if self.ball_size_px() < self.fetch_stop_px:
            lin = self.Kp_fwd * (self.fetch_stop_px - self.ball_size_px())
            lin = max(min(lin, self.max_lin), 0.0)

        abs_err = abs(err)
        if abs_err > self.ball_forward_slowdown_start_px:
            err_span = self.ball_forward_stop_error_px - self.ball_forward_slowdown_start_px
            err_ratio = max(min((abs_err - self.ball_forward_slowdown_start_px) / err_span, 1.0), 0.0)
            lin *= 1.0 - err_ratio

        if slow:
            ang *= self.slow_ang_scale
            lin *= self.slow_lin_scale

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
