#!/usr/bin/env python3
"""Publish serial-bound RealSense depth images on role-specific topics."""

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def import_runtime_modules():
    try:
        import numpy as np
        import pyrealsense2 as rs
        import yaml
    except ModuleNotFoundError as exc:
        print(
            f"Missing Python module: {exc.name}\n"
            "Install dependencies before running this node.\n"
            "Example:\n"
            "  python3 -m pip install pyrealsense2\n"
            "  sudo apt install python3-yaml",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    return np, rs, yaml


class RealSenseSerialMapper(Node):
    def __init__(self):
        super().__init__("realsense_serial_mapper")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("publish_rate_hz", 1.0)

        self._np, self._rs, self._yaml = import_runtime_modules()
        self._mapping_file = self.get_parameter("mapping_file").value
        self._config = self._load_config(self._mapping_file)
        self._bindings = [
            binding for binding in self._config["camera_bindings"]
            if bool(binding.get("enabled", True))
        ]
        self._stream = self._config["stream"]

        self._binding_pub = self.create_publisher(String, "~/camera_bindings", 10)
        self._depth_publishers = {}
        self._camera_info_publishers = {}
        self._pipelines = {}
        self._intrinsics = {}
        self._depth_scales = {}
        self._start_cameras()

        period = 1.0 / max(1.0, float(self._stream["depth_fps"]))
        self._timer = self.create_timer(period, self._publish_depth_maps)
        self._status_timer = self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _load_config(self, mapping_file):
        if not mapping_file:
            raise RuntimeError("Parameter 'mapping_file' is required")

        path = Path(mapping_file).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Mapping file does not exist: {path}")

        with path.open("r", encoding="utf-8") as stream:
            data = self._yaml.safe_load(stream) or {}

        bindings = data.get("camera_bindings", [])
        if isinstance(bindings, dict):
            bindings = bindings.get("real", [])
        if not isinstance(bindings, list):
            raise ValueError("'camera_bindings.real' must be a list")

        for binding in bindings:
            for key in ("role", "serial_no", "camera_name", "depth_topic", "camera_info_topic"):
                if key not in binding:
                    raise ValueError(f"Missing required key '{key}' in binding: {binding}")
            binding.setdefault("enabled", True)

        stream = data.get("stream", {})
        stream.setdefault("depth_width", 640)
        stream.setdefault("depth_height", 480)
        stream.setdefault("depth_fps", 60)
        stream.setdefault("max_range", 2.5)
        stream.setdefault(
            "fallback_profiles",
            [
                {"depth_width": 848, "depth_height": 480, "depth_fps": 60},
                {"depth_width": 640, "depth_height": 360, "depth_fps": 60},
                {"depth_width": 424, "depth_height": 240, "depth_fps": 60},
            ],
        )

        return {"camera_bindings": bindings, "stream": stream}

    def _connected_devices(self):
        context = self._rs.context()
        devices = {}
        for device in context.query_devices():
            serial = device.get_info(self._rs.camera_info.serial_number)
            name = device.get_info(self._rs.camera_info.name)
            devices[serial] = name
        return devices

    def _start_cameras(self):
        connected = self._connected_devices()
        for binding in self._bindings:
            serial = str(binding["serial_no"])
            if serial not in connected:
                self.get_logger().warn(
                    f"Skipping disconnected RealSense serial={serial} role={binding['role']}"
                )
                continue

            pipeline, active_profile = self._start_pipeline_with_fallback(serial, binding)
            if pipeline is None:
                continue

            role = binding["role"]
            self._pipelines[role] = pipeline
            self._intrinsics[role] = active_profile["intrinsics"]
            self._depth_scales[role] = active_profile["depth_scale"]
            self._depth_publishers[role] = self.create_publisher(
                Image, binding["depth_topic"], qos_profile_sensor_data
            )
            self._camera_info_publishers[role] = self.create_publisher(
                CameraInfo, binding["camera_info_topic"], qos_profile_sensor_data
            )

    def _candidate_profiles(self, binding):
        candidates = [
            {
                "depth_width": int(binding.get("depth_width", self._stream["depth_width"])),
                "depth_height": int(binding.get("depth_height", self._stream["depth_height"])),
                "depth_fps": int(binding.get("depth_fps", self._stream["depth_fps"])),
            }
        ]
        for profile in self._stream.get("fallback_profiles", []):
            candidate = {
                "depth_width": int(profile["depth_width"]),
                "depth_height": int(profile["depth_height"]),
                "depth_fps": int(profile["depth_fps"]),
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
            try:
                pipeline_profile = pipeline.start(config)
                depth_stream = pipeline_profile.get_stream(
                    self._rs.stream.depth
                ).as_video_stream_profile()
                depth_sensor = pipeline_profile.get_device().first_depth_sensor()
                active_profile = dict(profile)
                active_profile["intrinsics"] = depth_stream.get_intrinsics()
                active_profile["depth_scale"] = depth_sensor.get_depth_scale()
                return pipeline, active_profile
            except RuntimeError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Failed to start serial={serial} role={binding['role']} "
                    f"with depth={profile['depth_width']}x{profile['depth_height']}"
                    f"@{profile['depth_fps']}: {exc}"
                )

        self.get_logger().error(
            f"No usable depth profile for serial={serial} role={binding['role']}. "
            f"Last error: {last_error}"
        )
        self._log_supported_depth_profiles(serial)
        return None, None

    def _log_supported_depth_profiles(self, serial):
        context = self._rs.context()
        for device in context.query_devices():
            device_serial = device.get_info(self._rs.camera_info.serial_number)
            if device_serial != serial:
                continue

            profiles = []
            for sensor in device.query_sensors():
                for profile in sensor.get_stream_profiles():
                    if profile.stream_type() != self._rs.stream.depth:
                        continue
                    if profile.format() != self._rs.format.z16:
                        continue
                    try:
                        video_profile = profile.as_video_stream_profile()
                    except RuntimeError:
                        continue
                    profiles.append(
                        f"{video_profile.width()}x{video_profile.height()}@{profile.fps()}"
                    )

            unique_profiles = sorted(set(profiles))
            self.get_logger().error(
                f"Supported z16 depth profiles for serial={serial}: "
                + (", ".join(unique_profiles) if unique_profiles else "none")
            )
            return

    def _publish_depth_maps(self):
        for binding in self._bindings:
            role = binding["role"]
            pipeline = self._pipelines.get(role)
            if pipeline is None:
                continue

            frames = pipeline.poll_for_frames()
            if not frames:
                continue
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            stamp = self.get_clock().now().to_msg()
            frame_id = (
                binding.get("optical_frame")
                or binding.get("mount_frame")
                or binding["camera_name"]
            )
            depth = self._np.asanyarray(depth_frame.get_data()).astype(self._np.float32)
            depth *= float(self._depth_scales[role])

            image = self._depth_to_image_msg(depth, stamp, frame_id)
            camera_info = self._camera_info_msg(self._intrinsics[role], stamp, frame_id)
            self._depth_publishers[role].publish(image)
            self._camera_info_publishers[role].publish(camera_info)

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

    def _publish_status(self):
        connected = self._connected_devices()
        connected_serials = set(connected.keys())
        configured_serials = {str(binding["serial_no"]) for binding in self._bindings}

        resolved = []
        for binding in self._bindings:
            serial = str(binding["serial_no"])
            item = dict(binding)
            item["connected"] = serial in connected
            item["device_model"] = connected.get(serial, "")
            resolved.append(item)

        payload = {
            "connected_serials": sorted(connected_serials),
            "unconfigured_connected_serials": sorted(connected_serials - configured_serials),
            "missing_configured_serials": sorted(configured_serials - connected_serials),
            "bindings": resolved,
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._binding_pub.publish(msg)


def main():
    rclpy.init()
    node = RealSenseSerialMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for pipeline in node._pipelines.values():
            pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
