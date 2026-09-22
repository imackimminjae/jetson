#!/usr/bin/env python3
"""ROS 2 <-> PX4 EKF2 external-vision bridge.

Input frame (ROS/Motive):
  - ENU-like local map frame
  - +X east, +Y north, +Z up
  - yaw = 0 along +X, positive counter-clockwise toward +Y

MAVLink external pose sent to PX4:
  - default message: VISION_POSITION_ESTIMATE (best match for EV_CTRL=9)
  - optional message: ODOMETRY with frame_id=MAV_FRAME_LOCAL_NED and
    child_frame_id=MAV_FRAME_BODY_FRD
  - x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
  - yaw_ned = pi/2 - yaw_enu
  - ODOMETRY estimator_type defaults to VISION so PX4 routes it to
    vehicle_visual_odometry, which is the EKF2 external-vision input path.

PX4 fused output:
  - LOCAL_POSITION_NED + ATTITUDE are converted back to ENU and published as
    standard quaternion nav_msgs/Odometry.
  - The same aligned map-frame pose is optionally published as PoseStamped
    with wrapped yaw stored directly in pose.orientation.z.

This node intentionally does not arm or command actuators.
"""

import json
import math
import os
import struct
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

# ODOMETRY and TIMESYNC extension fields require MAVLink 2.
os.environ.setdefault("MAVLINK20", "1")

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - runtime dependency diagnostic
    raise RuntimeError(
        "pymavlink is required. Install with: python3 -m pip install --user pymavlink"
    ) from exc


@dataclass
class MapPose:
    stamp_usec: int
    receive_monotonic: float
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class Px4Parameter:
    name: str
    value: float
    value_type: str  # "int32" or "float"


@dataclass
class PendingParameter:
    spec: Px4Parameter
    attempts: int = 0
    last_send_monotonic: float = 0.0


class Px4EkfBridge(Node):
    """Bidirectional MAVLink bridge for PX4 EKF2 external vision aiding."""

    def __init__(self) -> None:
        super().__init__("px4_ekf_bridge")

        # ROS topics and frame conventions.
        self.declare_parameter("enable_motive_pose_input", True)
        self.declare_parameter("motive_pose_topic", "/motive/vehicle/pose")
        self.declare_parameter("output_odom_topic", "/px4/ekf_odom")
        self.declare_parameter("output_pose_topic", "/px4/sih/odom")
        self.declare_parameter("publish_yaw_scalar_pose", True)
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("output_child_frame_id", "base_link")
        self.declare_parameter("pose_orientation_mode", "yaw_scalar_z")
        self.declare_parameter("pose_yaw_offset_rad", 0.0)
        self.declare_parameter("pose_x_offset_m", 0.0)
        self.declare_parameter("pose_y_offset_m", 0.0)
        self.declare_parameter("pose_z_offset_m", 0.0)
        self.declare_parameter("pose_position_scale", 1.0)
        self.declare_parameter("pose_timeout_sec", 0.20)
        self.declare_parameter("zero_origin_on_first_pose", False)

        # External-vision MAVLink output.
        self.declare_parameter("external_vision_rate_hz", 50.0)
        self.declare_parameter("external_message_type", "vision_position_estimate")
        self.declare_parameter("external_vision_quality", 100)
        self.declare_parameter("external_estimator_type", "vision")
        self.declare_parameter("external_position_stddev_m", 0.03)
        self.declare_parameter("external_z_stddev_m", 0.10)
        self.declare_parameter("external_yaw_stddev_rad", 0.05)
        self.declare_parameter("use_pose_header_stamp", False)

        # MAVLink serial/transport parameters. pymavlink also accepts UDP URLs.
        self.declare_parameter("connection_url", "/dev/ttyACM0")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("source_system", 245)
        self.declare_parameter("source_component", 191)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("heartbeat_rate_hz", 1.0)
        self.declare_parameter("timesync_rate_hz", 5.0)
        self.declare_parameter("reconnect_period_sec", 2.0)

        # PX4 fused-state output stream.
        self.declare_parameter("request_output_streams", True)
        self.declare_parameter("output_stream_rate_hz", 30.0)
        self.declare_parameter("output_attitude_timeout_sec", 0.25)
        self.declare_parameter("align_output_to_motive_on_first_sample", False)

        # PX4 parameter protocol settings.
        self.declare_parameter("configure_px4_parameters", True)
        self.declare_parameter("parameter_ack_timeout_sec", 0.8)
        self.declare_parameter("parameter_max_attempts", 5)
        self.declare_parameter("parameter_apply_delay_sec", 1.0)
        self.declare_parameter(
            "px4_parameter_names",
            [
                "EKF2_EV_CTRL",
                "EKF2_EV_DELAY",
                "EKF2_EV_POS_X",
                "EKF2_EV_POS_Y",
                "EKF2_EV_POS_Z",
                "EKF2_MAG_TYPE",
                "EKF2_EV_NOISE_MD",
                "EKF2_EVP_NOISE",
                "EKF2_EVA_NOISE",
            ],
        )
        self.declare_parameter(
            "px4_parameter_values",
            [9.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.03, 0.05],
        )
        self.declare_parameter(
            "px4_parameter_types",
            [
                "int32",
                "float",
                "float",
                "float",
                "float",
                "int32",
                "int32",
                "float",
                "float",
            ],
        )

        self.enable_motive_pose_input = bool(
            self.get_parameter("enable_motive_pose_input").value
        )
        self.motive_pose_topic = str(self.get_parameter("motive_pose_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.publish_yaw_scalar_pose = bool(
            self.get_parameter("publish_yaw_scalar_pose").value
        )
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.output_child_frame_id = str(self.get_parameter("output_child_frame_id").value)
        self.pose_orientation_mode = str(self.get_parameter("pose_orientation_mode").value)
        # Keep Motive yaw unmodified. The project publishes vehicle-forward yaw
        # directly in PoseStamped.pose.orientation.z, so applying a rigid-body
        # yaw offset here can silently flip the EKF/planner heading.
        self.pose_yaw_offset_rad = 0.0
        self.pose_x_offset_m = float(self.get_parameter("pose_x_offset_m").value)
        self.pose_y_offset_m = float(self.get_parameter("pose_y_offset_m").value)
        self.pose_z_offset_m = float(self.get_parameter("pose_z_offset_m").value)
        self.pose_position_scale = float(self.get_parameter("pose_position_scale").value)
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.zero_origin_on_first_pose = bool(self.get_parameter("zero_origin_on_first_pose").value)

        self.external_vision_rate_hz = max(
            1.0, float(self.get_parameter("external_vision_rate_hz").value)
        )
        self.external_message_type = str(
            self.get_parameter("external_message_type").value
        ).strip().lower()
        self.external_vision_quality = int(self.get_parameter("external_vision_quality").value)
        self.external_estimator_type = str(
            self.get_parameter("external_estimator_type").value
        ).strip().lower()
        self.external_position_stddev_m = max(
            1.0e-4, float(self.get_parameter("external_position_stddev_m").value)
        )
        self.external_z_stddev_m = max(
            1.0e-4, float(self.get_parameter("external_z_stddev_m").value)
        )
        self.external_yaw_stddev_rad = max(
            1.0e-4, float(self.get_parameter("external_yaw_stddev_rad").value)
        )
        self.use_pose_header_stamp = bool(self.get_parameter("use_pose_header_stamp").value)

        self.connection_url = str(self.get_parameter("connection_url").value)
        self.baud = int(self.get_parameter("baud").value)
        self.source_system = int(self.get_parameter("source_system").value)
        self.source_component = int(self.get_parameter("source_component").value)
        self.configured_target_system = int(self.get_parameter("target_system").value)
        self.configured_target_component = int(self.get_parameter("target_component").value)
        self.target_system = self.configured_target_system
        self.target_component = self.configured_target_component
        self.heartbeat_rate_hz = max(0.2, float(self.get_parameter("heartbeat_rate_hz").value))
        self.timesync_rate_hz = max(0.2, float(self.get_parameter("timesync_rate_hz").value))
        self.reconnect_period_sec = max(
            0.5, float(self.get_parameter("reconnect_period_sec").value)
        )

        self.request_output_streams = bool(self.get_parameter("request_output_streams").value)
        self.output_stream_rate_hz = max(
            1.0, float(self.get_parameter("output_stream_rate_hz").value)
        )
        self.output_attitude_timeout_sec = max(
            0.05, float(self.get_parameter("output_attitude_timeout_sec").value)
        )
        self.align_output_to_motive_on_first_sample = bool(
            self.get_parameter("align_output_to_motive_on_first_sample").value
        )

        self.configure_px4_parameters = bool(
            self.get_parameter("configure_px4_parameters").value
        )
        self.parameter_ack_timeout_sec = max(
            0.1, float(self.get_parameter("parameter_ack_timeout_sec").value)
        )
        self.parameter_max_attempts = max(
            1, int(self.get_parameter("parameter_max_attempts").value)
        )
        self.parameter_apply_delay_sec = max(
            0.0, float(self.get_parameter("parameter_apply_delay_sec").value)
        )
        self.parameter_specs = self._load_parameter_specs()

        if self.pose_orientation_mode not in ("yaw_scalar_z", "quaternion"):
            raise ValueError(
                "pose_orientation_mode must be 'yaw_scalar_z' or 'quaternion'"
            )
        if self.external_message_type not in (
            "vision_position_estimate",
            "odometry",
        ):
            raise ValueError(
                "external_message_type must be 'vision_position_estimate' or 'odometry'"
            )
        estimator_type_map = {
            "vision": mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION,
            "vio": mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO,
            "mocap": mavutil.mavlink.MAV_ESTIMATOR_TYPE_MOCAP,
        }
        if self.external_estimator_type not in estimator_type_map:
            raise ValueError(
                "external_estimator_type must be 'vision', 'vio', or 'mocap'"
            )
        self.external_estimator_type_mavlink = estimator_type_map[
            self.external_estimator_type
        ]
        if self.external_estimator_type == "mocap":
            self.get_logger().warn(
                "external_estimator_type=mocap routes ODOMETRY to PX4's mocap "
                "topic, which EKF2 does not use for external-vision fusion. "
                "Use 'vision' for EKF2_EV_CTRL aiding."
            )

        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_sub = (
            self.create_subscription(
                PoseStamped, self.motive_pose_topic, self._pose_callback, pose_qos
            )
            if self.enable_motive_pose_input
            else None
        )
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.yaw_scalar_pose_pub = (
            self.create_publisher(PoseStamped, self.output_pose_topic, 10)
            if self.publish_yaw_scalar_pose
            else None
        )
        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.apply_params_srv = self.create_service(
            Trigger, "~/apply_px4_parameters", self._apply_parameters_service
        )
        self.request_streams_srv = self.create_service(
            Trigger, "~/request_streams", self._request_streams_service
        )
        self.reset_alignment_srv = self.create_service(
            Trigger, "~/reset_output_alignment", self._reset_alignment_service
        )

        self.master = None
        self.connected = False
        self.last_connect_attempt = 0.0
        self.last_heartbeat_rx = 0.0
        self.first_autopilot_heartbeat_monotonic = 0.0
        self.parameters_applied_for_connection = False
        self.streams_requested_for_connection = False

        self.latest_map_pose: Optional[MapPose] = None
        self.input_origin: Optional[Tuple[float, float, float]] = None
        self.external_reset_counter = 0
        self.external_tx_count = 0
        self.external_tx_error_count = 0

        self.latest_attitude_yaw_ned: Optional[float] = None
        self.latest_attitude_yawspeed_ned: Optional[float] = None
        self.latest_attitude_monotonic = 0.0
        self.latest_local_position_monotonic = 0.0
        self.output_rx_count = 0

        # Fixed SE(2) alignment from PX4 local ENU output into the Motive map.
        self.output_alignment_ready = not self.align_output_to_motive_on_first_sample
        self.output_alignment_yaw = 0.0
        self.output_alignment_tx = 0.0
        self.output_alignment_ty = 0.0

        self.param_queue: Deque[Px4Parameter] = deque()
        self.pending_param: Optional[PendingParameter] = None
        self.param_results: Dict[str, str] = {}

        self.rx_timer = self.create_timer(0.005, self._poll_mavlink)
        self.connect_timer = self.create_timer(0.25, self._connection_tick)
        self.ev_timer = (
            self.create_timer(
                1.0 / self.external_vision_rate_hz, self._send_external_odometry
            )
            if self.enable_motive_pose_input
            else None
        )
        self.heartbeat_timer = self.create_timer(
            1.0 / self.heartbeat_rate_hz, self._send_companion_heartbeat
        )
        self.timesync_timer = self.create_timer(
            1.0 / self.timesync_rate_hz, self._send_timesync_request
        )
        self.param_timer = self.create_timer(0.05, self._parameter_tick)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            "PX4 EKF bridge configured: input=%s odom=%s yaw_scalar_pose=%s "
            "connection=%s @ %d, "
            "EV=%.1f Hz (%s), ENU(+X east,+Y north,+Z up) <-> NED/FRD"
            % (
                self.motive_pose_topic if self.enable_motive_pose_input else "disabled",
                self.output_odom_topic,
                self.output_pose_topic if self.publish_yaw_scalar_pose else "disabled",
                self.connection_url,
                self.baud,
                self.external_vision_rate_hz,
                self.external_message_type,
            )
        )
        self.get_logger().warn(
            "Start with the vehicle disarmed and stationary. PX4 EKF parameter changes "
            "marked as reboot-required need a Pixhawk reboot before fusion is tested."
        )

    # ------------------------------------------------------------------
    # ROS input and coordinate conversion
    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_pi(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        # Standard ROS ENU yaw extraction.
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_to_quaternion_enu(yaw: float) -> Tuple[float, float, float, float]:
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    @staticmethod
    def _enu_to_ned_position(x_e: float, y_n: float, z_u: float) -> Tuple[float, float, float]:
        return (y_n, x_e, -z_u)

    @staticmethod
    def _ned_to_enu_position(x_n: float, y_e: float, z_d: float) -> Tuple[float, float, float]:
        return (y_e, x_n, -z_d)

    @classmethod
    def _enu_yaw_to_ned(cls, yaw_enu: float) -> float:
        return cls._wrap_pi(0.5 * math.pi - yaw_enu)

    @classmethod
    def _ned_yaw_to_enu(cls, yaw_ned: float) -> float:
        return cls._wrap_pi(0.5 * math.pi - yaw_ned)

    def _pose_callback(self, msg: PoseStamped) -> None:
        x = self.pose_position_scale * float(msg.pose.position.x) + self.pose_x_offset_m
        y = self.pose_position_scale * float(msg.pose.position.y) + self.pose_y_offset_m
        z = self.pose_position_scale * float(msg.pose.position.z) + self.pose_z_offset_m

        if self.pose_orientation_mode == "yaw_scalar_z":
            yaw = float(msg.pose.orientation.z)
        else:
            yaw = self._yaw_from_quaternion(
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            )
        yaw = self._wrap_pi(yaw)

        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            self.get_logger().warn("Rejected non-finite Motive pose")
            return

        if self.zero_origin_on_first_pose and self.input_origin is None:
            self.input_origin = (x, y, z)
            self.get_logger().warn(
                "External-vision origin zeroed at first pose. This changes the map origin; "
                "do not use it unless the path is transformed by the same offset."
            )
        if self.input_origin is not None:
            x -= self.input_origin[0]
            y -= self.input_origin[1]
            z -= self.input_origin[2]

        stamp_usec = self._pose_stamp_usec(msg)
        self.latest_map_pose = MapPose(
            stamp_usec=stamp_usec,
            receive_monotonic=time.monotonic(),
            x=x,
            y=y,
            z=z,
            yaw=yaw,
        )

    def _pose_stamp_usec(self, msg: PoseStamped) -> int:
        if self.use_pose_header_stamp:
            sec = int(msg.header.stamp.sec)
            nanosec = int(msg.header.stamp.nanosec)
            if sec != 0 or nanosec != 0:
                return sec * 1_000_000 + nanosec // 1_000
        return time.time_ns() // 1_000

    # ------------------------------------------------------------------
    # MAVLink connection and receive loop
    # ------------------------------------------------------------------
    def _connection_tick(self) -> None:
        now_mono = time.monotonic()
        if self.master is None:
            if now_mono - self.last_connect_attempt >= self.reconnect_period_sec:
                self.last_connect_attempt = now_mono
                self._open_connection()
            return

        # Treat a long heartbeat outage as a disconnected autopilot while keeping
        # the serial object alive; pymavlink may recover automatically.
        if self.connected and now_mono - self.last_heartbeat_rx > 5.0:
            self.get_logger().warn("PX4 heartbeat timeout; waiting for reconnection")
            self.connected = False
            self.parameters_applied_for_connection = False
            self.streams_requested_for_connection = False
            self.pending_param = None
            self.param_queue.clear()

        if self.connected:
            elapsed = now_mono - self.first_autopilot_heartbeat_monotonic
            if (
                self.configure_px4_parameters
                and not self.parameters_applied_for_connection
                and elapsed >= self.parameter_apply_delay_sec
            ):
                self._start_parameter_application()
                self.parameters_applied_for_connection = True
            if self.request_output_streams and not self.streams_requested_for_connection:
                self._request_output_message_intervals()
                self.streams_requested_for_connection = True

    def _open_connection(self) -> None:
        try:
            self.get_logger().info(
                "Opening MAVLink connection %s (baud=%d)"
                % (self.connection_url, self.baud)
            )
            self.master = mavutil.mavlink_connection(
                self.connection_url,
                baud=self.baud,
                source_system=self.source_system,
                source_component=self.source_component,
                autoreconnect=True,
                robust_parsing=True,
                dialect="common",
            )
            self.connected = False
            self.parameters_applied_for_connection = False
            self.streams_requested_for_connection = False
        except Exception as exc:  # serial errors are environment-specific
            self.get_logger().error("Failed to open MAVLink connection: %s" % exc)
            self.master = None

    def _close_connection(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
        self.master = None
        self.connected = False

    def _poll_mavlink(self) -> None:
        if self.master is None:
            return
        try:
            for _ in range(100):
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                if msg.get_type() == "BAD_DATA":
                    continue
                self._handle_mavlink_message(msg)
        except Exception as exc:
            self.get_logger().error("MAVLink receive error: %s" % exc)
            self._close_connection()

    def _handle_mavlink_message(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            autopilot = int(getattr(msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID))
            if autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                was_connected = self.connected
                self.connected = True
                self.last_heartbeat_rx = time.monotonic()
                self.target_system = int(msg.get_srcSystem()) or self.configured_target_system
                self.target_component = int(msg.get_srcComponent()) or self.configured_target_component
                if not was_connected:
                    self.first_autopilot_heartbeat_monotonic = time.monotonic()
                    self.parameters_applied_for_connection = False
                    self.streams_requested_for_connection = False
                    self.get_logger().info(
                        "PX4 heartbeat detected: target=%d:%d"
                        % (self.target_system, self.target_component)
                    )
            return

        if msg_type == "TIMESYNC":
            self._handle_timesync(msg)
            return

        if msg_type == "PARAM_VALUE":
            self._handle_param_value(msg)
            return

        if msg_type == "PARAM_ERROR":
            self.get_logger().error("PX4 PARAM_ERROR: %s" % msg)
            return

        if msg_type == "COMMAND_ACK":
            command = int(getattr(msg, "command", -1))
            result = int(getattr(msg, "result", -1))
            if command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                self.get_logger().debug(
                    "SET_MESSAGE_INTERVAL ACK result=%d" % result
                )
            return

        if msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            severity = int(getattr(msg, "severity", 255))
            if severity <= mavutil.mavlink.MAV_SEVERITY_WARNING:
                self.get_logger().warn("PX4: %s" % str(text).rstrip("\x00"))
            else:
                self.get_logger().info("PX4: %s" % str(text).rstrip("\x00"))
            return

        if msg_type == "ATTITUDE":
            yaw = float(getattr(msg, "yaw", float("nan")))
            if math.isfinite(yaw):
                self.latest_attitude_yaw_ned = self._wrap_pi(yaw)
                yawspeed = float(getattr(msg, "yawspeed", float("nan")))
                self.latest_attitude_yawspeed_ned = (
                    yawspeed if math.isfinite(yawspeed) else None
                )
                self.latest_attitude_monotonic = time.monotonic()
            return

        if msg_type == "LOCAL_POSITION_NED":
            self.latest_local_position_monotonic = time.monotonic()
            self._publish_fused_odometry(msg)
            return

    # ------------------------------------------------------------------
    # External odometry transmission
    # ------------------------------------------------------------------
    def _send_external_odometry(self) -> None:
        if not self.connected or self.master is None or self.latest_map_pose is None:
            return
        pose = self.latest_map_pose
        age = time.monotonic() - pose.receive_monotonic
        if self.pose_timeout_sec > 0.0 and age > self.pose_timeout_sec:
            return

        x_ned, y_ned, z_ned = self._enu_to_ned_position(pose.x, pose.y, pose.z)
        yaw_ned = self._enu_yaw_to_ned(pose.yaw)
        q_ned_frd = [
            math.cos(0.5 * yaw_ned),
            0.0,
            0.0,
            math.sin(0.5 * yaw_ned),
        ]

        pos_var = self.external_position_stddev_m ** 2
        z_var = self.external_z_stddev_m ** 2
        yaw_var = self.external_yaw_stddev_rad ** 2
        pose_covariance = [0.0] * 21
        pose_covariance[0] = pos_var   # x-x
        pose_covariance[6] = pos_var   # y-y
        pose_covariance[11] = z_var    # z-z
        pose_covariance[15] = 1.0      # roll-roll, unused for EV_CTRL=9
        pose_covariance[18] = 1.0      # pitch-pitch, unused for EV_CTRL=9
        pose_covariance[20] = yaw_var  # yaw-yaw

        nan = float("nan")
        velocity_covariance = [0.0] * 21
        velocity_covariance[0] = nan

        try:
            if self.external_message_type == "vision_position_estimate":
                try:
                    self.master.mav.vision_position_estimate_send(
                        int(pose.stamp_usec),
                        float(x_ned),
                        float(y_ned),
                        float(z_ned),
                        0.0,
                        0.0,
                        float(yaw_ned),
                        pose_covariance,
                        int(self.external_reset_counter) & 0xFF,
                    )
                except TypeError:
                    # Compatibility with older generated pymavlink dialects that
                    # omit MAVLink 2 extension fields (covariance/reset_counter).
                    self.master.mav.vision_position_estimate_send(
                        int(pose.stamp_usec),
                        float(x_ned),
                        float(y_ned),
                        float(z_ned),
                        0.0,
                        0.0,
                        float(yaw_ned),
                    )
            else:
                self.master.mav.odometry_send(
                    int(pose.stamp_usec),
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    float(x_ned),
                    float(y_ned),
                    float(z_ned),
                    q_ned_frd,
                    nan,
                    nan,
                    nan,
                    nan,
                    nan,
                    nan,
                    pose_covariance,
                    velocity_covariance,
                    int(self.external_reset_counter) & 0xFF,
                    self.external_estimator_type_mavlink,
                    max(-1, min(100, int(self.external_vision_quality))),
                )
            self.external_tx_count += 1
        except Exception as exc:
            self.external_tx_error_count += 1
            self.get_logger().error(
                "Failed to send MAVLink external pose (%s): %s"
                % (self.external_message_type, exc)
            )

    # ------------------------------------------------------------------
    # PX4 fused output
    # ------------------------------------------------------------------
    def _publish_fused_odometry(self, msg) -> None:
        if self.latest_attitude_yaw_ned is None:
            return
        if time.monotonic() - self.latest_attitude_monotonic > self.output_attitude_timeout_sec:
            return

        values = [
            float(getattr(msg, "x", float("nan"))),
            float(getattr(msg, "y", float("nan"))),
            float(getattr(msg, "z", float("nan"))),
            float(getattr(msg, "vx", float("nan"))),
            float(getattr(msg, "vy", float("nan"))),
            float(getattr(msg, "vz", float("nan"))),
        ]
        if not all(math.isfinite(v) for v in values):
            return

        x_enu, y_enu, z_enu = self._ned_to_enu_position(values[0], values[1], values[2])
        vx_enu, vy_enu, vz_enu = self._ned_to_enu_position(values[3], values[4], values[5])
        yaw_enu = self._ned_yaw_to_enu(self.latest_attitude_yaw_ned)

        if self.align_output_to_motive_on_first_sample and not self.output_alignment_ready:
            if self.latest_map_pose is None:
                return
            pose_age = time.monotonic() - self.latest_map_pose.receive_monotonic
            if self.pose_timeout_sec > 0.0 and pose_age > self.pose_timeout_sec:
                return
            self._initialize_output_alignment(
                x_enu, y_enu, yaw_enu, self.latest_map_pose
            )

        if self.output_alignment_ready:
            c = math.cos(self.output_alignment_yaw)
            s = math.sin(self.output_alignment_yaw)
            x_aligned = c * x_enu - s * y_enu + self.output_alignment_tx
            y_aligned = s * x_enu + c * y_enu + self.output_alignment_ty
            vx_aligned = c * vx_enu - s * vy_enu
            vy_aligned = s * vx_enu + c * vy_enu
            yaw_aligned = self._wrap_pi(yaw_enu + self.output_alignment_yaw)
        else:
            x_aligned, y_aligned = x_enu, y_enu
            vx_aligned, vy_aligned = vx_enu, vy_enu
            yaw_aligned = yaw_enu

        yaw_aligned = self._wrap_pi(yaw_aligned)
        qx, qy, qz, qw = self._yaw_to_quaternion_enu(yaw_aligned)
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.output_frame_id
        odom.child_frame_id = self.output_child_frame_id
        odom.pose.pose.position.x = x_aligned
        odom.pose.pose.position.y = y_aligned
        odom.pose.pose.position.z = z_enu
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        # nav_msgs/Odometry defines twist in child_frame_id. Convert the
        # PX4 earth-frame ENU velocity into ROS body FLU (forward, left, up).
        cy = math.cos(yaw_aligned)
        sy = math.sin(yaw_aligned)
        v_forward = cy * vx_aligned + sy * vy_aligned
        v_left = -sy * vx_aligned + cy * vy_aligned
        odom.twist.twist.linear.x = v_forward
        odom.twist.twist.linear.y = v_left
        odom.twist.twist.linear.z = vz_enu
        if self.latest_attitude_yawspeed_ned is not None:
            # NED/FRD yaw rate is positive about +Z down. ROS FLU is positive
            # about +Z up, hence the sign inversion.
            odom.twist.twist.angular.z = -self.latest_attitude_yawspeed_ned

        # Conservative covariance for downstream logging. The tracking controller
        # currently reads the state fields directly and does not consume covariance.
        pos_var = self.external_position_stddev_m ** 2
        yaw_var = self.external_yaw_stddev_rad ** 2
        odom.pose.covariance[0] = pos_var
        odom.pose.covariance[7] = pos_var
        odom.pose.covariance[14] = self.external_z_stddev_m ** 2
        odom.pose.covariance[21] = 1.0
        odom.pose.covariance[28] = 1.0
        odom.pose.covariance[35] = yaw_var

        self.odom_pub.publish(odom)
        if self.yaw_scalar_pose_pub is not None:
            scalar_pose = PoseStamped()
            scalar_pose.header = odom.header
            scalar_pose.pose.position.x = x_aligned
            scalar_pose.pose.position.y = y_aligned
            scalar_pose.pose.position.z = z_enu
            # Project convention: this is deliberately not a quaternion.
            # Consumers must read orientation.z directly as a wrapped map-frame yaw.
            scalar_pose.pose.orientation.x = 0.0
            scalar_pose.pose.orientation.y = 0.0
            scalar_pose.pose.orientation.z = yaw_aligned
            scalar_pose.pose.orientation.w = 0.0
            self.yaw_scalar_pose_pub.publish(scalar_pose)
        self.output_rx_count += 1

    def _initialize_output_alignment(
        self, px4_x: float, px4_y: float, px4_yaw: float, motive: MapPose
    ) -> None:
        theta = self._wrap_pi(motive.yaw - px4_yaw)
        c = math.cos(theta)
        s = math.sin(theta)
        self.output_alignment_yaw = theta
        self.output_alignment_tx = motive.x - (c * px4_x - s * px4_y)
        self.output_alignment_ty = motive.y - (s * px4_x + c * px4_y)
        self.output_alignment_ready = True
        self.get_logger().warn(
            "PX4 output aligned to Motive map using first sample: yaw=%.3f rad, "
            "translation=(%.3f, %.3f). Keep the vehicle stationary during startup."
            % (
                self.output_alignment_yaw,
                self.output_alignment_tx,
                self.output_alignment_ty,
            )
        )

    # ------------------------------------------------------------------
    # PX4 parameter protocol
    # ------------------------------------------------------------------
    def _load_parameter_specs(self) -> List[Px4Parameter]:
        names = list(self.get_parameter("px4_parameter_names").value)
        values = list(self.get_parameter("px4_parameter_values").value)
        types = list(self.get_parameter("px4_parameter_types").value)
        if not (len(names) == len(values) == len(types)):
            raise ValueError(
                "px4_parameter_names, px4_parameter_values and px4_parameter_types "
                "must have equal lengths"
            )
        result: List[Px4Parameter] = []
        for name, value, value_type in zip(names, values, types):
            clean_name = str(name).strip()
            clean_type = str(value_type).strip().lower()
            if not clean_name:
                continue
            if len(clean_name) > 16:
                raise ValueError("PX4 parameter name exceeds 16 chars: %s" % clean_name)
            if clean_type not in ("int32", "float"):
                raise ValueError(
                    "PX4 parameter type must be int32 or float: %s=%s"
                    % (clean_name, clean_type)
                )
            result.append(Px4Parameter(clean_name, float(value), clean_type))
        return result

    def _start_parameter_application(self) -> None:
        if not self.connected:
            return
        self.param_queue = deque(self.parameter_specs)
        self.pending_param = None
        self.param_results = {}
        self.get_logger().warn(
            "Applying %d PX4 parameters. Do not arm. A Pixhawk reboot is required "
            "after reboot-required parameters are changed." % len(self.param_queue)
        )

    def _parameter_tick(self) -> None:
        if not self.connected or self.master is None:
            return
        now_mono = time.monotonic()
        if self.pending_param is not None:
            elapsed = now_mono - self.pending_param.last_send_monotonic
            if elapsed >= self.parameter_ack_timeout_sec:
                if self.pending_param.attempts >= self.parameter_max_attempts:
                    spec = self.pending_param.spec
                    self.param_results[spec.name] = "timeout"
                    self.get_logger().error(
                        "PX4 parameter timeout: %s after %d attempts"
                        % (spec.name, self.pending_param.attempts)
                    )
                    self.pending_param = None
                else:
                    self._send_pending_parameter()
            return

        if self.param_queue:
            self.pending_param = PendingParameter(self.param_queue.popleft())
            self._send_pending_parameter()
            return

        if self.param_results and len(self.param_results) == len(self.parameter_specs):
            failures = [k for k, v in self.param_results.items() if v != "ok"]
            if failures:
                self.get_logger().error(
                    "PX4 parameter application completed with failures: %s"
                    % ", ".join(failures)
                )
            else:
                self.get_logger().warn(
                    "PX4 parameters acknowledged. Reboot the Pixhawk before testing EKF2 fusion."
                )
            # Empty marker prevents printing the completion message repeatedly.
            self.param_results = {}

    def _send_pending_parameter(self) -> None:
        if self.pending_param is None or self.master is None:
            return
        spec = self.pending_param.spec
        try:
            if spec.value_type == "int32":
                wire_value = struct.unpack("<f", struct.pack("<i", int(round(spec.value))))[0]
                mav_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
            else:
                wire_value = float(spec.value)
                mav_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            self.master.mav.param_set_send(
                self.target_system,
                self.target_component,
                spec.name.encode("ascii"),
                wire_value,
                mav_type,
            )
            self.pending_param.attempts += 1
            self.pending_param.last_send_monotonic = time.monotonic()
            self.get_logger().info(
                "PX4 PARAM_SET %s=%s (%s), attempt %d"
                % (spec.name, spec.value, spec.value_type, self.pending_param.attempts)
            )
        except Exception as exc:
            self.get_logger().error("PX4 PARAM_SET failed for %s: %s" % (spec.name, exc))

    @staticmethod
    def _normalize_param_id(param_id) -> str:
        if isinstance(param_id, bytes):
            raw = param_id.decode("ascii", errors="ignore")
        else:
            raw = str(param_id)
        return raw.rstrip("\x00")

    @staticmethod
    def _decode_param_value(msg) -> float:
        param_type = int(getattr(msg, "param_type", mavutil.mavlink.MAV_PARAM_TYPE_REAL32))
        raw_value = float(getattr(msg, "param_value", float("nan")))
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return float(struct.unpack("<i", struct.pack("<f", raw_value))[0])
        return raw_value

    def _handle_param_value(self, msg) -> None:
        if self.pending_param is None:
            return
        name = self._normalize_param_id(getattr(msg, "param_id", ""))
        spec = self.pending_param.spec
        if name != spec.name:
            return
        actual = self._decode_param_value(msg)
        tolerance = 0.0 if spec.value_type == "int32" else max(1.0e-5, abs(spec.value) * 1.0e-4)
        if math.isfinite(actual) and abs(actual - spec.value) <= tolerance:
            self.param_results[spec.name] = "ok"
            self.get_logger().info("PX4 parameter ACK %s=%s" % (spec.name, actual))
        else:
            self.param_results[spec.name] = "mismatch"
            self.get_logger().error(
                "PX4 parameter mismatch %s: requested=%s received=%s"
                % (spec.name, spec.value, actual)
            )
        self.pending_param = None

    # ------------------------------------------------------------------
    # MAVLink heartbeat, timesync and output stream requests
    # ------------------------------------------------------------------
    def _send_companion_heartbeat(self) -> None:
        if self.master is None:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception:
            pass

    def _send_timesync_request(self) -> None:
        if not self.connected or self.master is None:
            return
        now_ns = time.time_ns()
        try:
            # Recent MAVLink 2 dialects include target_system/component extensions.
            self.master.mav.timesync_send(
                0, now_ns, self.target_system, self.target_component
            )
        except TypeError:
            self.master.mav.timesync_send(0, now_ns)
        except Exception:
            pass

    def _handle_timesync(self, msg) -> None:
        if self.master is None:
            return
        tc1 = int(getattr(msg, "tc1", 0))
        ts1 = int(getattr(msg, "ts1", 0))
        if tc1 != 0:
            return  # Response to our request; PX4/transport handles offset filtering.
        now_ns = time.time_ns()
        target_system = int(msg.get_srcSystem())
        target_component = int(msg.get_srcComponent())
        try:
            self.master.mav.timesync_send(
                now_ns, ts1, target_system, target_component
            )
        except TypeError:
            self.master.mav.timesync_send(now_ns, ts1)
        except Exception:
            pass

    def _request_output_message_intervals(self) -> None:
        if not self.connected or self.master is None:
            return
        interval_us = int(round(1_000_000.0 / self.output_stream_rate_hz))
        for msg_id, label in (
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, "LOCAL_POSITION_NED"),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, "ATTITUDE"),
        ):
            try:
                self.master.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    float(interval_us),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
                self.get_logger().info(
                    "Requested PX4 %s at %.1f Hz" % (label, self.output_stream_rate_hz)
                )
            except Exception as exc:
                self.get_logger().error("Failed to request %s: %s" % (label, exc))

    # ------------------------------------------------------------------
    # ROS services and status
    # ------------------------------------------------------------------
    def _apply_parameters_service(self, request, response):
        del request
        if not self.connected:
            response.success = False
            response.message = "PX4 heartbeat not available"
            return response
        self._start_parameter_application()
        self.parameters_applied_for_connection = True
        response.success = True
        response.message = "PX4 parameter application started; monitor logs for ACKs"
        return response

    def _request_streams_service(self, request, response):
        del request
        if not self.connected:
            response.success = False
            response.message = "PX4 heartbeat not available"
            return response
        self._request_output_message_intervals()
        response.success = True
        response.message = "PX4 output stream requests sent"
        return response

    def _reset_alignment_service(self, request, response):
        del request
        self.output_alignment_ready = not self.align_output_to_motive_on_first_sample
        self.output_alignment_yaw = 0.0
        self.output_alignment_tx = 0.0
        self.output_alignment_ty = 0.0
        self.external_reset_counter = (self.external_reset_counter + 1) & 0xFF
        response.success = True
        response.message = (
            "Output alignment reset; next valid PX4/Motive sample will establish alignment"
        )
        return response

    def _publish_status(self) -> None:
        now_mono = time.monotonic()
        pose_age = None
        if self.latest_map_pose is not None:
            pose_age = now_mono - self.latest_map_pose.receive_monotonic
        att_age = None
        if self.latest_attitude_monotonic > 0.0:
            att_age = now_mono - self.latest_attitude_monotonic
        local_pos_age = None
        if self.latest_local_position_monotonic > 0.0:
            local_pos_age = now_mono - self.latest_local_position_monotonic
        payload = {
            "connected": self.connected,
            "target_system": self.target_system,
            "target_component": self.target_component,
            "pose_age_sec": pose_age,
            "attitude_age_sec": att_age,
            "local_position_age_sec": local_pos_age,
            "external_message_type": self.external_message_type,
            "external_estimator_type": self.external_estimator_type,
            "external_odometry_tx": self.external_tx_count,
            "external_odometry_tx_errors": self.external_tx_error_count,
            "fused_odometry_rx": self.output_rx_count,
            "output_yaw_scalar_pose_topic": (
                self.output_pose_topic if self.publish_yaw_scalar_pose else None
            ),
            "output_alignment_ready": self.output_alignment_ready,
            "output_alignment_yaw_rad": self.output_alignment_yaw,
            "output_alignment_translation": [
                self.output_alignment_tx,
                self.output_alignment_ty,
            ],
            "pending_px4_parameter": (
                self.pending_param.spec.name if self.pending_param is not None else None
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._close_connection()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4EkfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Humble's default SIGINT handler can shut the context down before
        # rclpy.spin() raises KeyboardInterrupt. Avoid a second shutdown call.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
