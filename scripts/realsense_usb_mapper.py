#!/usr/bin/python3
"""Publish USB-port-bound RealSense RGB-D images on role-specific topics."""

import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener

from autonomy.msg import AutonomyState


def import_yaml_module():
    try:
        import yaml
    except ModuleNotFoundError as exc:
        print(
            f"Missing Python module: {exc.name}\n"
            "Install dependencies before running this node.\n"
            "Example:\n"
            "  sudo apt install python3-yaml",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    return yaml


def import_realsense_runtime_modules():
    try:
        import numpy as np
        import pyrealsense2 as rs
    except ModuleNotFoundError as exc:
        print(
            f"Missing Python module: {exc.name}\n"
            "Install dependencies before running cameras.\n"
            "Example:\n"
            "  python3 -m pip install pyrealsense2",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    return np, rs


def parse_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "y", "on"):
            return True
        if normalized in ("false", "0", "no", "n", "off"):
            return False
    return bool(value)


class RealSenseUsbMapper(Node):
    def __init__(self):
        super().__init__("drive_mapper_node")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("operation_mode", "drive")
        self.declare_parameter("simulation", False)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("respect_autonomy_mode", False)
        self.declare_parameter("autonomy_status_topic", "/autonomy_manager/status")
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("publish_stabilized_tf", True)
        self.declare_parameter("low_pass_alpha", 0.08)
        self.declare_parameter("min_accel_norm", 3.0)
        self.declare_parameter("max_accel_norm", 20.0)
        self.declare_parameter("roll_sign", -1.0)
        self.declare_parameter("pitch_sign", -1.0)

        self._yaml = import_yaml_module()
        self._np = None
        self._rs = None
        self._mapping_file = self.get_parameter("mapping_file").value
        self._operation_mode = str(self.get_parameter("operation_mode").value).strip().lower()
        self._simulation = parse_bool(self.get_parameter("simulation").value, default=False)
        self._respect_autonomy_mode = parse_bool(
            self.get_parameter("respect_autonomy_mode").value,
            default=False,
        )
        self._autonomy_status_topic = str(self.get_parameter("autonomy_status_topic").value)
        self._has_autonomy_state = False
        self._autonomy_mode = AutonomyState.IDLE
        self._config = self._load_config(self._mapping_file)
        self._robot = self._config["robot"]
        self._bindings = self._select_bindings_for_operation_mode(
            self._config["camera_bindings"],
            self._config["operation_modes"],
            self._operation_mode,
        )
        self._stream = self._config["stream"]
        self._temperature_config = self._config["camera_temperature"]
        self._frame_prefix = self._robot_frame_prefix()
        self._base_frame_id = self._robot_frame("base_link", "base_link")
        self._stabilized_frame_id = self._robot_frame("base_stabilized_link", "base_stabilized")
        self._front_binding = self._find_binding("front")
        self._imu_topic = self._binding_imu_topic(self._front_binding) if self._front_binding else ""
        self._imu_frame_id = self._merge_frame(self._binding_frame(self._front_binding, ""))
        self._publish_static_tf = parse_bool(self.get_parameter("publish_static_tf").value, default=True)
        self._publish_stabilized_tf = parse_bool(
            self.get_parameter("publish_stabilized_tf").value,
            default=True,
        )
        self._low_pass_alpha = min(1.0, max(0.0, float(self.get_parameter("low_pass_alpha").value)))
        self._min_accel_norm = max(0.1, float(self.get_parameter("min_accel_norm").value))
        self._max_accel_norm = max(self._min_accel_norm, float(self.get_parameter("max_accel_norm").value))
        self._roll_sign = float(self.get_parameter("roll_sign").value)
        self._pitch_sign = float(self.get_parameter("pitch_sign").value)
        self._roll = 0.0
        self._pitch = 0.0
        self._has_attitude_estimate = False
        self._received_imu = False
        self._last_imu_frame = ""
        self._last_transform_frame = ""

        self._binding_pub = self.create_publisher(String, "~/camera_bindings", 10)
        self._temperature_pub = self.create_publisher(String, "~/temperatures", 10)
        self._heartbeat_pub = self.create_publisher(String, "/autonomy/heartbeat/drive_mapper_node", 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._depth_publishers = {}
        self._camera_info_publishers = {}
        self._color_publishers = {}
        self._color_camera_info_publishers = {}
        self._pipelines = {}
        self._aligners = {}
        self._depth_intrinsics = {}
        self._color_intrinsics = {}
        self._depth_scales = {}
        self._active_profiles = {}
        self._depth_sensors = {}
        self._overheat_shutdown_requested = False
        self._last_temperature_log_ns = 0
        self._logged_depth_publishers = set()
        self._logged_color_publishers = set()
        self._autonomy_sub = None
        self._imu_sub = None
        if self._respect_autonomy_mode:
            self._autonomy_sub = self.create_subscription(
                AutonomyState,
                self._autonomy_status_topic,
                self._on_autonomy_state,
                10,
            )
        if self._publish_static_tf:
            self._publish_urdf_static_tf()
        if self._publish_stabilized_tf and self._imu_topic:
            self._imu_sub = self.create_subscription(
                Imu,
                self._imu_topic,
                self._on_imu,
                qos_profile_sensor_data,
            )
        if self._processing_active() and not self._simulation:
            self._start_cameras()

        period = 1.0 / max(1.0, float(self._stream["depth_fps"]))
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(period, self._publish_depth_maps, clock=steady_clock)
        self._status_timer = self.create_timer(1.0, self._publish_status, clock=steady_clock)
        self._heartbeat_timer = self.create_timer(0.5, self._publish_heartbeat, clock=steady_clock)
        self._tf_timer = self.create_timer(0.01, self._publish_stabilized_tf_timer, clock=steady_clock)
        self._publish_status()

    def _on_autonomy_state(self, msg):
        was_active = self._processing_active()
        self._autonomy_mode = msg.mode
        self._has_autonomy_state = True
        is_active = self._processing_active()

        if is_active and not was_active and not self._simulation:
            self._start_cameras()
        elif was_active and not is_active and not self._simulation:
            self._stop_cameras()

    def _processing_active(self):
        if not self._respect_autonomy_mode:
            return True
        if not self._has_autonomy_state:
            return False
        return self._autonomy_mode == AutonomyState.DRIVE

    def _ensure_runtime_modules(self):
        if self._np is None or self._rs is None:
            self._np, self._rs = import_realsense_runtime_modules()

    def _load_config(self, mapping_file):
        if not mapping_file:
            raise RuntimeError("Parameter 'mapping_file' is required")

        path = Path(mapping_file).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Mapping file does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            data = self._yaml.safe_load(stream) or {}

        robot = self._normalize_robot_config(data.get("robot", {}) or {})

        bindings = data.get("camera_bindings", [])
        if isinstance(bindings, dict):
            bindings = bindings.get("simulation" if self._simulation else "real", [])
        if not isinstance(bindings, list):
            raise ValueError("'camera_bindings' must be a list")

        for binding in bindings:
            binding.setdefault("enabled", True)
            if not parse_bool(binding.get("enabled", True), default=True):
                continue

            for key in ("role", "camera_name"):
                if key not in binding:
                    raise ValueError(f"Missing required key '{key}' in binding: {binding}")
            if not self._simulation and "usb_port_id" not in binding and "serial_no" not in binding:
                raise ValueError(
                    "Missing required key 'usb_port_id' in binding "
                    f"(legacy 'serial_no' is still accepted): {binding}"
                )

            camera_name = str(binding.get("camera_name", "")).strip("/")
            if camera_name:
                base_topic = f"{str(robot.get('topic_prefix', '/f4')).rstrip('/')}/{camera_name}"
                binding.setdefault("depth_topic", f"{base_topic}/depth/image_rect_raw")
                binding.setdefault("camera_info_topic", f"{base_topic}/depth/camera_info")
                binding.setdefault("imu_topic", f"{base_topic}/imu")
                binding.setdefault(
                    "frame",
                    binding.get("mount_frame") or binding.get("optical_frame") or camera_name,
                )

        stream = data.get("stream", {})
        stream.setdefault("depth_width", 640)
        stream.setdefault("depth_height", 480)
        stream.setdefault("depth_fps", 60)
        stream.setdefault("color_width", stream["depth_width"])
        stream.setdefault("color_height", stream["depth_height"])
        stream.setdefault("color_fps", min(30, int(stream["depth_fps"])))
        stream.setdefault("max_range", 2.5)
        stream.setdefault(
            "fallback_profiles",
            [
                {
                    "depth_width": 848,
                    "depth_height": 480,
                    "depth_fps": 60,
                    "color_width": 848,
                    "color_height": 480,
                    "color_fps": 30,
                },
                {
                    "depth_width": 640,
                    "depth_height": 360,
                    "depth_fps": 60,
                    "color_width": 640,
                    "color_height": 360,
                    "color_fps": 30,
                },
                {
                    "depth_width": 424,
                    "depth_height": 240,
                    "depth_fps": 60,
                    "color_width": 424,
                    "color_height": 240,
                    "color_fps": 30,
                },
            ],
        )

        temperature = data.get("camera_temperature", {}) or {}
        temperature.setdefault("enabled", True)
        temperature.setdefault("publish_rate_hz", 1.0)
        temperature.setdefault("danger_celsius", 70.0)
        temperature.setdefault("shutdown_on_danger", True)

        operation_modes = data.get("operation_modes") or self._default_operation_modes()

        return {
            "camera_bindings": bindings,
            "operation_modes": operation_modes,
            "stream": stream,
            "camera_temperature": temperature,
            "robot": robot,
        }

    def _normalize_robot_config(self, robot):
        model = str(robot.get("model", "f4")).strip("/") or "f4"
        robot = dict(robot)
        robot["model"] = model
        robot.setdefault("namespace", model)
        robot.setdefault("frame_prefix", f"{model}/")
        robot.setdefault("topic_prefix", f"/{model}")
        robot.setdefault("urdf_path", f"src/autonomy/resources/urdf/{model}.urdf")
        robot.setdefault("base_link", "base_link")
        robot.setdefault("base_footprint_link", "base_footprint")
        robot.setdefault("base_stabilized_link", "base_stabilized")
        robot.setdefault("lidar_link", "lidar_link")
        robot.setdefault("map_frame", "map")
        robot.setdefault("odom_frame", "odom")
        return robot

    def _default_operation_modes(self):
        return {
            "drive": {"camera_roles": ["front", "rear"], "require_roles": []},
            "adas": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
            "fsd": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
            "tracking": {"camera_roles": ["front", "rear", "adas"], "require_roles": ["adas"]},
        }

    def _robot_frame_prefix(self):
        prefix = str(self._robot.get("frame_prefix", "")).strip("/")
        if not prefix:
            prefix = str(self._robot.get("namespace", self._robot.get("name", ""))).strip("/")
        return f"{prefix}/" if prefix else ""

    def _robot_frame(self, key, default):
        link = str(self._robot.get(key, default)).strip().lstrip("/")
        if "/" in link:
            return link
        return self._frame_prefix + link

    def _merge_frame(self, frame):
        text = str(frame or "").strip().lstrip("/")
        if not text or "/" in text:
            return text
        return self._frame_prefix + text

    def _find_binding(self, role):
        for binding in self._bindings:
            if str(binding.get("role", "")).strip().lower() == role:
                return binding
        return None

    def _binding_frame(self, binding, default=None):
        if binding is None:
            return "" if default is None else default
        fallback = binding.get("camera_name", "") if default is None else default
        return (
            binding.get("frame")
            or binding.get("imu_frame")
            or binding.get("optical_frame")
            or binding.get("mount_frame")
            or fallback
        )

    def _topic_prefix(self):
        return str(self._robot.get("topic_prefix", "/f4")).rstrip("/")

    def _camera_base_topic(self, binding):
        camera_name = str(binding.get("camera_name", "front_camera")).strip("/")
        return f"{self._topic_prefix()}/{camera_name}"

    def _binding_imu_topic(self, binding):
        if binding is None:
            return ""
        return str(binding.get("imu_topic", f"{self._camera_base_topic(binding)}/imu"))

    def _resolve_mapping_path(self, value):
        if not value:
            return ""
        path = Path(str(value)).expanduser()
        if path.is_absolute():
            return str(path)
        source_root = Path(__file__).resolve().parents[1]
        if str(value).startswith("src/autonomy/"):
            return str(source_root / str(value)[len("src/autonomy/"):])
        return str(source_root / str(value))

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _publish_urdf_static_tf(self):
        urdf_path = self._resolve_mapping_path(self._robot.get("urdf_path", ""))
        if not urdf_path:
            self.get_logger().warn("No robot.urdf_path configured; mapper cannot publish URDF static TF")
            return
        path = Path(urdf_path)
        if not path.exists():
            self.get_logger().warn(f"URDF does not exist; mapper cannot publish static TF: {path}")
            return

        transforms = []
        root = ET.parse(path).getroot()
        for joint in root.findall("joint"):
            if joint.get("type") != "fixed":
                continue
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue
            parent_frame = self._merge_frame(parent.get("link", ""))
            child_frame = self._merge_frame(child.get("link", ""))
            if not parent_frame or not child_frame or parent_frame == child_frame:
                continue
            origin = joint.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(item) for item in origin.get("xyz", "0 0 0").split()]
                rpy = [float(item) for item in origin.get("rpy", "0 0 0").split()]
            transforms.append(self._transform_msg(parent_frame, child_frame, xyz, rpy))

        seen_aliases = set()
        for binding in self._bindings:
            for frame in (
                self._binding_frame(binding, ""),
                binding.get("rgb_frame", ""),
                binding.get("rgb_optical_frame", ""),
                binding.get("color_optical_frame", ""),
            ):
                child = str(frame).strip().lstrip("/")
                if not child or "/" in child:
                    continue
                parent = self._merge_frame(child)
                if parent == child or (parent, child) in seen_aliases:
                    continue
                seen_aliases.add((parent, child))
                transforms.append(self._transform_msg(parent, child, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]))

        if transforms:
            self._static_tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f"Published {len(transforms)} mapper static TF transforms from URDF")

    def _transform_msg(self, parent, child, xyz, rpy):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = self._quaternion_from_rpy(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _on_imu(self, msg):
        accel = self._acceleration_in_base_frame(msg)
        if accel is None:
            return
        norm = math.sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2])
        if norm < self._min_accel_norm or norm > self._max_accel_norm:
            return
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.hypot(accel[1], accel[2]))
        if not self._has_attitude_estimate:
            self._roll = roll
            self._pitch = pitch
            self._has_attitude_estimate = True
        else:
            self._roll = (1.0 - self._low_pass_alpha) * self._roll + self._low_pass_alpha * roll
            self._pitch = (1.0 - self._low_pass_alpha) * self._pitch + self._low_pass_alpha * pitch
        self._received_imu = True
        self._last_imu_frame = msg.header.frame_id

    def _acceleration_in_base_frame(self, msg):
        source_frame = self._imu_frame_id or msg.header.frame_id
        self._last_transform_frame = source_frame
        if not source_frame:
            return (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        if source_frame == self._base_frame_id:
            return (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame_id,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for IMU TF {source_frame} -> {self._base_frame_id}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        return self._rotate_vector(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ),
        )

    def _rotate_vector(self, vector, quaternion):
        x, y, z = vector
        qx, qy, qz, qw = quaternion
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + (qy * tz - qz * ty),
            y + qw * ty + (qz * tx - qx * tz),
            z + qw * tz + (qx * ty - qy * tx),
        )

    def _publish_stabilized_tf_timer(self):
        if not self._publish_stabilized_tf or not self._has_attitude_estimate:
            return
        msg = self._transform_msg(
            self._base_frame_id,
            self._stabilized_frame_id,
            [0.0, 0.0, 0.0],
            [self._roll_sign * self._roll, self._pitch_sign * self._pitch, 0.0],
        )
        self._tf_broadcaster.sendTransform(msg)

    def _select_bindings_for_operation_mode(self, bindings, operation_modes, operation_mode):
        if operation_mode not in operation_modes:
            valid = ", ".join(sorted(operation_modes.keys()))
            raise RuntimeError(
                f"Unsupported operation_mode '{operation_mode}'. Valid modes: {valid}"
            )

        mode = operation_modes.get(operation_mode) or {}
        camera_roles = [str(role) for role in mode.get("camera_roles", [])]
        role_allowlist = set(camera_roles)

        selected = []
        for binding in bindings:
            role = str(binding.get("role", ""))
            if camera_roles and role not in role_allowlist:
                continue
            if parse_bool(binding.get("enabled", True), default=True):
                selected.append(binding)

        return selected

    def _connected_devices(self):
        context = self._rs.context()
        devices = {}
        for device in context.query_devices():
            serial = device.get_info(self._rs.camera_info.serial_number)
            devices[serial] = {
                "serial": serial,
                "name": self._safe_device_info(device, self._rs.camera_info.name),
                "physical_port": self._safe_device_info(
                    device, getattr(self._rs.camera_info, "physical_port", None)
                ),
            }
            devices[serial]["usb_port_id"] = self._usb_port_id(devices[serial]["physical_port"])
        return devices

    def _safe_device_info(self, device, info):
        if info is None:
            return ""
        try:
            return device.get_info(info)
        except RuntimeError:
            return ""

    def _usb_port_id(self, physical_port):
        text = str(physical_port or "")
        matches = re.findall(r"(?<![\w.])\d+-\d+(?:\.\d+)*(?=[:/]|$)", text)
        if matches:
            return matches[-1]

        # Some platforms expose a shorter value. Keep a stable non-empty suffix
        # rather than forcing users back to camera serial numbers.
        parts = [part for part in text.split("/") if part]
        return parts[-1].split(":")[0] if parts else ""

    def _resolve_binding_device(self, binding, connected, *, quiet=False):
        usb_port_id = str(binding.get("usb_port_id", "")).strip()
        if usb_port_id:
            matches = [
                info for info in connected.values()
                if self._matches_usb_port(usb_port_id, info)
            ]
            if len(matches) == 1:
                return matches[0]
            if len(matches) > 1:
                if not quiet:
                    serials = ", ".join(sorted(info["serial"] for info in matches))
                    self.get_logger().error(
                        f"USB port id '{usb_port_id}' for role={binding['role']} "
                        f"matches multiple RealSense devices: {serials}"
                    )
                return None

            if not quiet:
                self.get_logger().warn(
                    f"Skipping disconnected RealSense usb_port_id={usb_port_id} "
                    f"role={binding['role']}"
                )
            return None

        serial = str(binding.get("serial_no", "")).strip()
        if serial and serial in connected:
            return connected[serial]

        if not quiet:
            self.get_logger().warn(
                f"Skipping unresolved RealSense role={binding['role']} "
                "because neither usb_port_id nor connected legacy serial_no matched"
            )
        return None

    def _matches_usb_port(self, configured_port, device_info):
        configured = str(configured_port).strip()
        return configured in {
            str(device_info.get("usb_port_id", "")).strip(),
            str(device_info.get("physical_port", "")).strip(),
        } or configured in str(device_info.get("physical_port", ""))

    def _start_cameras(self):
        if self._pipelines:
            return
        self._ensure_runtime_modules()
        connected = self._connected_devices()
        for binding in self._bindings:
            device_info = self._resolve_binding_device(binding, connected)
            if device_info is None:
                continue

            serial = device_info["serial"]
            pipeline, active_profile = self._start_pipeline_with_fallback(serial, binding)
            if pipeline is None:
                continue

            role = binding["role"]
            binding["resolved_serial_no"] = serial
            binding["resolved_usb_port_id"] = device_info.get("usb_port_id", "")
            binding["physical_port"] = device_info.get("physical_port", "")
            self._pipelines[role] = pipeline
            if "rgb_topic" in binding:
                self._aligners[role] = self._rs.align(self._rs.stream.color)
            self._depth_intrinsics[role] = active_profile["depth_intrinsics"]
            self._depth_scales[role] = active_profile["depth_scale"]
            self._active_profiles[role] = active_profile
            self._depth_sensors[role] = active_profile["depth_sensor"]
            self._depth_publishers[role] = self.create_publisher(
                Image, binding["depth_topic"], qos_profile_sensor_data
            )
            self._camera_info_publishers[role] = self.create_publisher(
                CameraInfo, binding["camera_info_topic"], qos_profile_sensor_data
            )
            if "rgb_topic" in binding:
                self._color_intrinsics[role] = active_profile["color_intrinsics"]
                self._color_publishers[role] = self.create_publisher(
                    Image, binding["rgb_topic"], qos_profile_sensor_data
                )
                rgb_camera_info_topic = binding.get("rgb_camera_info_topic") or binding.get(
                    "color_camera_info_topic"
                )
                if rgb_camera_info_topic:
                    self._color_camera_info_publishers[role] = self.create_publisher(
                        CameraInfo, rgb_camera_info_topic, qos_profile_sensor_data
                    )

    def _stop_cameras(self):
        for pipeline in self._pipelines.values():
            try:
                pipeline.stop()
            except RuntimeError:
                pass
        self._pipelines.clear()
        self._aligners.clear()
        self._depth_intrinsics.clear()
        self._color_intrinsics.clear()
        self._depth_scales.clear()
        self._active_profiles.clear()
        self._depth_sensors.clear()
        self._depth_publishers.clear()
        self._camera_info_publishers.clear()
        self._color_publishers.clear()
        self._color_camera_info_publishers.clear()
        self._logged_depth_publishers.clear()
        self._logged_color_publishers.clear()

    def _candidate_profiles(self, binding):
        candidates = [
            {
                "depth_width": int(binding.get("depth_width", self._stream["depth_width"])),
                "depth_height": int(binding.get("depth_height", self._stream["depth_height"])),
                "depth_fps": int(binding.get("depth_fps", self._stream["depth_fps"])),
                "color_width": int(binding.get("color_width", self._stream["color_width"])),
                "color_height": int(binding.get("color_height", self._stream["color_height"])),
                "color_fps": int(binding.get("color_fps", self._stream["color_fps"])),
            }
        ]
        for profile in self._stream.get("fallback_profiles", []):
            candidate = {
                "depth_width": int(profile["depth_width"]),
                "depth_height": int(profile["depth_height"]),
                "depth_fps": int(profile["depth_fps"]),
                "color_width": int(profile.get("color_width", profile["depth_width"])),
                "color_height": int(profile.get("color_height", profile["depth_height"])),
                "color_fps": int(profile.get("color_fps", min(30, int(profile["depth_fps"])))),
            }
            if candidate not in candidates:
                candidates.append(candidate)
        return candidates

    def _start_pipeline_with_fallback(self, serial, binding):
        last_error = None
        for profile in self._candidate_profiles(binding):
            pipeline = self._rs.pipeline()
            config = self._rs.config()
            config.enable_device(serial)
            config.enable_stream(
                self._rs.stream.depth,
                profile["depth_width"],
                profile["depth_height"],
                self._rs.format.z16,
                profile["depth_fps"],
            )
            if "rgb_topic" in binding:
                config.enable_stream(
                    self._rs.stream.color,
                    profile["color_width"],
                    profile["color_height"],
                    self._rs.format.rgb8,
                    profile["color_fps"],
                )
            try:
                pipeline_profile = pipeline.start(config)
                depth_stream = pipeline_profile.get_stream(
                    self._rs.stream.depth
                ).as_video_stream_profile()
                depth_sensor = pipeline_profile.get_device().first_depth_sensor()
                active_profile = dict(profile)
                active_profile["depth_intrinsics"] = depth_stream.get_intrinsics()
                if "rgb_topic" in binding:
                    color_stream = pipeline_profile.get_stream(
                        self._rs.stream.color
                    ).as_video_stream_profile()
                    active_profile["color_intrinsics"] = color_stream.get_intrinsics()
                active_profile["depth_scale"] = depth_sensor.get_depth_scale()
                active_profile["depth_sensor"] = depth_sensor
                return pipeline, active_profile
            except RuntimeError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Failed to start serial={serial} role={binding['role']} "
                    f"with depth={profile['depth_width']}x{profile['depth_height']}"
                    f"@{profile['depth_fps']} color={profile['color_width']}x"
                    f"{profile['color_height']}@{profile['color_fps']}: {exc}"
                )

        self.get_logger().error(
            f"No usable RGB-D profile for serial={serial} role={binding['role']}. "
            f"Last error: {last_error}"
        )
        self._log_supported_profiles(serial)
        return None, None

    def _log_supported_profiles(self, serial):
        context = self._rs.context()
        for device in context.query_devices():
            device_serial = device.get_info(self._rs.camera_info.serial_number)
            if device_serial != serial:
                continue

            profiles = []
            color_profiles = []
            for sensor in device.query_sensors():
                for profile in sensor.get_stream_profiles():
                    stream_type = profile.stream_type()
                    if stream_type == self._rs.stream.depth and profile.format() != self._rs.format.z16:
                        continue
                    if stream_type == self._rs.stream.color and profile.format() != self._rs.format.rgb8:
                        continue
                    if stream_type not in (self._rs.stream.depth, self._rs.stream.color):
                        continue
                    try:
                        video_profile = profile.as_video_stream_profile()
                    except RuntimeError:
                        continue
                    item = (
                        f"{video_profile.width()}x{video_profile.height()}@{profile.fps()}"
                    )
                    if stream_type == self._rs.stream.depth:
                        profiles.append(item)
                    else:
                        color_profiles.append(item)

            unique_profiles = sorted(set(profiles))
            self.get_logger().error(
                f"Supported z16 depth profiles for serial={serial}: "
                + (", ".join(unique_profiles) if unique_profiles else "none")
            )
            unique_color_profiles = sorted(set(color_profiles))
            self.get_logger().error(
                f"Supported rgb8 color profiles for serial={serial}: "
                + (", ".join(unique_color_profiles) if unique_color_profiles else "none")
            )
            return

    def _publish_depth_maps(self):
        if not self._processing_active():
            return

        for binding in self._bindings:
            role = binding["role"]
            pipeline = self._pipelines.get(role)
            if pipeline is None:
                continue

            frames = pipeline.poll_for_frames()
            if not frames:
                continue
            aligner = self._aligners.get(role)
            if aligner is not None:
                frames = aligner.process(frames)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            color_frame = frames.get_color_frame() if role in self._color_publishers else None

            stamp = self.get_clock().now().to_msg()
            frame_id = self._binding_frame(binding)
            depth = self._np.asanyarray(depth_frame.get_data()).astype(self._np.float32)
            depth *= float(self._depth_scales[role])

            image = self._depth_to_image_msg(depth, stamp, frame_id)
            depth_intrinsics = self._frame_intrinsics(depth_frame, self._depth_intrinsics[role])
            camera_info = self._camera_info_msg(depth_intrinsics, stamp, frame_id)
            self._depth_publishers[role].publish(image)
            self._camera_info_publishers[role].publish(camera_info)

            if role not in self._logged_depth_publishers:
                active_profile = self._active_profiles.get(role, {})
                self.get_logger().info(
                    f"Publishing depth role={role} topic={binding['depth_topic']} "
                    f"resolution={image.width}x{image.height} "
                    f"hz={active_profile.get('depth_fps', self._stream['depth_fps'])}"
                )
                self._logged_depth_publishers.add(role)

            if color_frame:
                color_frame_id = (
                    binding.get("rgb_frame")
                    or binding.get("color_frame")
                    or binding.get("rgb_optical_frame")
                    or binding.get("color_optical_frame")
                    or frame_id
                )
                color = self._np.asanyarray(color_frame.get_data())
                color_image = self._color_to_image_msg(color, stamp, color_frame_id)
                self._color_publishers[role].publish(color_image)
                color_info_pub = self._color_camera_info_publishers.get(role)
                if color_info_pub is not None:
                    color_intrinsics = self._frame_intrinsics(
                        color_frame,
                        self._color_intrinsics[role],
                    )
                    color_info_pub.publish(
                        self._camera_info_msg(color_intrinsics, stamp, color_frame_id)
                    )
                if role not in self._logged_color_publishers:
                    active_profile = self._active_profiles.get(role, {})
                    self.get_logger().info(
                        f"Publishing RGB role={role} topic={binding['rgb_topic']} "
                        f"resolution={color_image.width}x{color_image.height} "
                        f"hz={active_profile.get('color_fps', self._stream['color_fps'])}"
                    )
                    self._logged_color_publishers.add(role)

    def _depth_to_image_msg(self, depth, stamp, frame_id):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(depth.shape[0])
        msg.width = int(depth.shape[1])
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = msg.width * 4
        msg.data = depth.astype(self._np.float32, copy=False).tobytes()
        return msg

    def _color_to_image_msg(self, color, stamp, frame_id):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(color.shape[0])
        msg.width = int(color.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = msg.width * 3
        msg.data = color.astype(self._np.uint8, copy=False).tobytes()
        return msg

    def _frame_intrinsics(self, frame, fallback):
        try:
            return frame.profile.as_video_stream_profile().get_intrinsics()
        except RuntimeError:
            return fallback

    def _camera_info_msg(self, intrinsics, stamp, frame_id):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = intrinsics.height
        msg.width = intrinsics.width
        msg.distortion_model = "plumb_bob"
        msg.d = list(intrinsics.coeffs)
        msg.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _temperature_enabled(self):
        return parse_bool(self._temperature_config.get("enabled", True), default=True)

    def _temperature_log_interval_ns(self):
        hz = max(0.1, float(self._temperature_config.get("publish_rate_hz", 1.0)))
        return int(1.0e9 / hz)

    def _sensor_temperatures(self, sensor):
        if sensor is None or self._rs is None:
            return {}

        option_names = {
            "asic": "asic_temperature",
            "projector": "projector_temperature",
        }
        readings = {}
        for name, option_attr in option_names.items():
            option = getattr(self._rs.option, option_attr, None)
            if option is None:
                continue
            try:
                if sensor.supports(option):
                    readings[name] = float(sensor.get_option(option))
            except RuntimeError as exc:
                readings[name] = f"unavailable:{exc}"
        return readings

    def _temperature_payload(self):
        readings = {}
        if not self._temperature_enabled():
            return readings

        for binding in self._bindings:
            role = binding["role"]
            sensor = self._depth_sensors.get(role)
            temperatures = self._sensor_temperatures(sensor)
            if not temperatures:
                continue

            readings[role] = {
                "camera_name": binding.get("camera_name", ""),
                "serial_no": binding.get("resolved_serial_no", ""),
                "usb_port_id": binding.get("resolved_usb_port_id", ""),
                "physical_port": binding.get("physical_port", ""),
                "temperatures_c": temperatures,
            }
        return readings

    def _publish_temperature_payload(self, readings):
        msg = String()
        msg.data = json.dumps(readings, sort_keys=True)
        self._temperature_pub.publish(msg)

    def _log_temperature_payload(self, readings):
        if not readings:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_temperature_log_ns < self._temperature_log_interval_ns():
            return
        self._last_temperature_log_ns = now_ns

        parts = []
        for role, item in sorted(readings.items()):
            temps = item.get("temperatures_c", {})
            temp_text = ", ".join(
                f"{name}={value:.1f}C" if isinstance(value, float) else f"{name}={value}"
                for name, value in sorted(temps.items())
            )
            parts.append(
                f"{role}({item.get('camera_name', '')}, usb={item.get('usb_port_id', '')}): {temp_text}"
            )
        self.get_logger().info("RealSense temperatures: " + " | ".join(parts))

    def _check_temperature_shutdown(self, readings):
        if self._overheat_shutdown_requested:
            return
        if not parse_bool(self._temperature_config.get("shutdown_on_danger", True), default=True):
            return

        danger_c = float(self._temperature_config.get("danger_celsius", 70.0))
        for role, item in readings.items():
            for name, value in item.get("temperatures_c", {}).items():
                if not isinstance(value, float) or value < danger_c:
                    continue
                self._overheat_shutdown_requested = True
                self.get_logger().fatal(
                    f"RealSense overheat: role={role} sensor={name} "
                    f"temperature={value:.1f}C >= danger_celsius={danger_c:.1f}C. "
                    "Stopping cameras and shutting down autonomy RealSense mapper."
                )
                self._stop_cameras()
                rclpy.shutdown()
                return

    def _publish_status(self):
        if not self._processing_active():
            msg = String()
            msg.data = json.dumps({"status": "standby", "bindings": []}, sort_keys=True)
            self._binding_pub.publish(msg)
            self._publish_temperature_payload({})
            return

        if self._simulation:
            resolved = []
            for binding in self._bindings:
                base = self._camera_base_topic(binding)
                item = dict(binding)
                item["depth_topic"] = str(binding.get("depth_topic", f"{base}/depth/image_rect_raw"))
                item["camera_info_topic"] = str(binding.get("camera_info_topic", f"{base}/depth/camera_info"))
                item["imu_topic"] = str(binding.get("imu_topic", f"{base}/imu"))
                item["connected"] = True
                resolved.append(item)
            msg = String()
            msg.data = json.dumps({
                "status": "ready",
                "runtime": "simulation",
                "base_frame_id": self._base_frame_id,
                "stabilized_frame_id": self._stabilized_frame_id,
                "imu_topic": self._imu_topic,
                "imu_frame_id": self._imu_frame_id,
                "bindings": resolved,
            }, sort_keys=True)
            self._binding_pub.publish(msg)
            self._publish_temperature_payload({})
            return

        connected = self._connected_devices()
        temperature_readings = self._temperature_payload()
        connected_serials = set(connected.keys())
        connected_usb_ports = {
            info["usb_port_id"] for info in connected.values() if info.get("usb_port_id")
        }
        configured_usb_ports = {
            str(binding.get("usb_port_id", "")).strip()
            for binding in self._bindings
            if str(binding.get("usb_port_id", "")).strip()
        }

        resolved = []
        for binding in self._bindings:
            device_info = self._resolve_binding_device(binding, connected, quiet=True)
            item = dict(binding)
            item["connected"] = device_info is not None
            item["resolved_serial_no"] = device_info["serial"] if device_info else ""
            item["resolved_usb_port_id"] = device_info.get("usb_port_id", "") if device_info else ""
            item["physical_port"] = device_info.get("physical_port", "") if device_info else ""
            item["device_model"] = device_info.get("name", "") if device_info else ""
            if item.get("role") in temperature_readings:
                item["temperatures_c"] = temperature_readings[item["role"]]["temperatures_c"]
            resolved.append(item)

        payload = {
            "connected_serials": sorted(connected_serials),
            "connected_usb_port_ids": sorted(connected_usb_ports),
            "unconfigured_connected_usb_port_ids": sorted(connected_usb_ports - configured_usb_ports),
            "missing_configured_usb_port_ids": sorted(configured_usb_ports - connected_usb_ports),
            "temperature_danger_celsius": float(self._temperature_config.get("danger_celsius", 70.0)),
            "temperatures": temperature_readings,
            "bindings": resolved,
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._binding_pub.publish(msg)
        self._publish_temperature_payload(temperature_readings)
        self._log_temperature_payload(temperature_readings)
        self._check_temperature_shutdown(temperature_readings)

    def _publish_heartbeat(self):
        msg = String()
        if not self._processing_active():
            msg.data = "standby"
        elif self._publish_stabilized_tf and not self._has_attitude_estimate:
            msg.data = "waiting_for_imu"
        elif self._simulation:
            msg.data = "ready:simulation_tf"
        else:
            msg.data = "ready" if self._pipelines else "degraded:no_active_cameras"
        if self._has_attitude_estimate:
            msg.data += f":imu_frame={self._last_imu_frame}:transform_frame={self._last_transform_frame}"
        self._heartbeat_pub.publish(msg)


def main():
    rclpy.init()
    node = RealSenseUsbMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._stop_cameras()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
