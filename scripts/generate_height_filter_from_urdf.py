#!/usr/bin/env python3
"""
Generate IsaacLab RayCasterFOV sensor code from:
  1. camera binding YAML
  2. robot URDF

Generated output files:
  - ray_caster_fov.py
  - ray_caster_fov_cfg.py

Concept:
  RayCasterFOV does NOT modify ray_hits_w.
  It only computes camera-FOV valid masks from actual ray hit points.

  Existing RayCaster height grid:
      ray_hits_w: [num_envs, num_rays, 3]

  FOV mask calculation:
      ray_hits_w
        -> world frame to base_link frame
        -> base_link frame to each camera optical frame
        -> depth range + h_fov + v_fov check
        -> valid_mask: [num_envs, num_rays]

  Final observation masking should be done in my_mdp.height_scan_fov(),
  not inside the sensor itself.

Usage:
  python3 generate_height_filter_from_urdf.py

Optional:
  python3 generate_height_filter_from_urdf.py \
    --output-dir src/autonomy/resources/sensors
"""

import argparse
import math
import sys
import textwrap
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml


@dataclass
class CameraSpec:
    model: str
    h_fov_deg: float
    v_fov_deg: float
    min_depth: float
    max_depth: float


@dataclass
class CameraBinding:
    role: str
    model: str
    mount_frame: str
    optical_frame: str
    depth_topic: str
    camera_info_topic: str
    enabled: bool = True
    usb_port_id: str = ""
    camera_name: str = ""


@dataclass
class ResolvedCamera:
    role: str
    model: str
    mount_frame: str
    optical_frame: str
    T_base_optical: np.ndarray
    spec: CameraSpec


@dataclass
class ElevationGridExample:
    name: str
    resolution: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    width: int
    height: int
    isaac_size_x: float
    isaac_size_y: float
    isaac_offset_x: float
    isaac_offset_y: float


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


# Approximate RealSense depth FOV values.
# 실기에서는 CameraInfo의 K로 FOV를 계산하는 것이 가장 정확하지만,
# 이 generator는 offline URDF+YAML 기반이므로 모델별 기본값을 사용한다.
CAMERA_SPEC_DB: Dict[str, CameraSpec] = {
    "D435": CameraSpec(
        model="D435",
        h_fov_deg=87.0,
        v_fov_deg=58.0,
        min_depth=0.10,
        max_depth=2.50,
    ),
    "D435I": CameraSpec(
        model="D435I",
        h_fov_deg=87.0,
        v_fov_deg=58.0,
        min_depth=0.10,
        max_depth=2.50,
    ),
    "D455": CameraSpec(
        model="D455",
        h_fov_deg=86.0,
        v_fov_deg=57.0,
        min_depth=0.20,
        max_depth=6.00,
    ),
}


def normalize_frame_name(name: str) -> str:
    """Normalize frame names for URDF lookup."""
    return str(name).strip().lstrip("/")


def normalize_model_name(model: str) -> str:
    return str(model).strip().upper().replace("-", "").replace("_", "")


def parse_bool(value, default: bool = True) -> bool:
    """Parse YAML-style booleans, including quoted true/false strings."""
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


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=np.float64,
    )

    Ry = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=np.float64,
    )

    Rz = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    return Rz @ Ry @ Rx


def xyz_rpy_to_matrix(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    T[:3, 3] = xyz.astype(np.float64)
    return T


def parse_origin(joint_elem: ET.Element) -> np.ndarray:
    origin = joint_elem.find("origin")
    xyz = np.zeros(3, dtype=np.float64)
    rpy = np.zeros(3, dtype=np.float64)

    if origin is not None:
        xyz_text = origin.get("xyz")
        rpy_text = origin.get("rpy")

        if xyz_text:
            xyz = np.array([float(v) for v in xyz_text.split()], dtype=np.float64)

        if rpy_text:
            rpy = np.array([float(v) for v in rpy_text.split()], dtype=np.float64)

    return xyz_rpy_to_matrix(xyz, rpy)


def load_urdf_tree(urdf_path: Path):
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")

    root = ET.parse(str(urdf_path)).getroot()

    link_names = set()
    for link in root.findall("link"):
        name = link.get("name")
        if name:
            link_names.add(normalize_frame_name(name))

    child_to_parent = {}
    parent_to_children = {}

    for joint in root.findall("joint"):
        joint_type = joint.get("type", "")
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")

        if parent_elem is None or child_elem is None:
            continue

        parent = normalize_frame_name(parent_elem.get("link", ""))
        child = normalize_frame_name(child_elem.get("link", ""))
        T_parent_child = parse_origin(joint)

        child_to_parent[child] = {
            "parent": parent,
            "type": joint_type,
            "T_parent_child": T_parent_child,
        }
        parent_to_children.setdefault(parent, []).append(child)

    return link_names, child_to_parent, parent_to_children


def get_T_base_target(
    base_link: str,
    target_link: str,
    child_to_parent: Dict[str, dict],
    allow_non_fixed: bool = False,
) -> np.ndarray:
    """
    Compute T_base_target using URDF parent chain.

    주의:
      - fixed joint chain이면 정확하다.
      - 중간에 revolute/prismatic joint가 있으면 현재 joint angle 없이는 정확하지 않다.
    """
    base_link = normalize_frame_name(base_link)
    target_link = normalize_frame_name(target_link)

    if base_link == target_link:
        return np.eye(4, dtype=np.float64)

    chain = []
    current = target_link
    visited = set()

    while current != base_link:
        if current in visited:
            raise RuntimeError(f"URDF chain has a cycle near link: {current}")
        visited.add(current)

        if current not in child_to_parent:
            raise ValueError(
                f"Cannot find parent chain from base_link='{base_link}' "
                f"to target_link='{target_link}'. Missing parent for '{current}'."
            )

        item = child_to_parent[current]
        joint_type = item["type"]

        if joint_type != "fixed" and not allow_non_fixed:
            raise ValueError(
                f"Non-fixed joint found in chain to camera frame '{target_link}': "
                f"link='{current}', joint_type='{joint_type}'. "
                f"Camera frames should usually be attached through fixed joints. "
                f"Use --allow-non-fixed only if you intentionally accept URDF zero pose."
            )

        chain.append(item["T_parent_child"])
        current = item["parent"]

    T_base_target = np.eye(4, dtype=np.float64)
    for T_parent_child in reversed(chain):
        T_base_target = T_base_target @ T_parent_child

    return T_base_target


def load_mapping(mapping_file: Path) -> Tuple[List[CameraBinding], dict]:
    if not mapping_file.exists():
        raise FileNotFoundError(f"Mapping YAML does not exist: {mapping_file}")

    with mapping_file.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    bindings_raw = data.get("camera_bindings", [])
    if isinstance(bindings_raw, dict):
        bindings_raw = bindings_raw.get("simulation", bindings_raw.get("real", []))
    if not isinstance(bindings_raw, list):
        raise ValueError("'camera_bindings.simulation' must be a list")

    stream = data.get("stream", {}) or {}
    stream.setdefault("depth_width", 640)
    stream.setdefault("depth_height", 480)
    stream.setdefault("depth_fps", 60)
    stream.setdefault("max_range", 2.5)

    bindings = []
    required = [
        "role",
        "model",
        "mount_frame",
        "optical_frame",
        "depth_topic",
        "camera_info_topic",
    ]

    for raw in bindings_raw:
        if not isinstance(raw, dict):
            raise ValueError(f"Each camera binding must be a map/dict: {raw}")

        enabled = parse_bool(raw.get("enabled", True), default=True)
        if not enabled:
            continue

        for key in required:
            if key not in raw:
                raise ValueError(
                    f"Missing required key '{key}' in enabled binding: {raw}"
                )

        bindings.append(
            CameraBinding(
                role=str(raw["role"]),
                model=str(raw["model"]),
                mount_frame=normalize_frame_name(raw["mount_frame"]),
                optical_frame=normalize_frame_name(raw["optical_frame"]),
                depth_topic=str(raw["depth_topic"]),
                camera_info_topic=str(raw["camera_info_topic"]),
                enabled=enabled,
                usb_port_id=str(raw.get("usb_port_id", "")),
                camera_name=str(raw.get("camera_name", "")),
            )
        )

    if not bindings:
        raise ValueError(
            "No enabled camera bindings found in mapping YAML. "
            "Set enabled: true for at least one camera under camera_bindings."
        )

    return bindings, stream


def make_elevation_grid_example(name: str, grid: dict, elevation_config: Path) -> ElevationGridExample:
    required = ["resolution", "x_min", "x_max", "y_min", "y_max"]
    for key in required:
        if key not in grid:
            raise ValueError(f"Missing {name} grid key '{key}' in: {elevation_config}")

    resolution = float(grid["resolution"])
    x_min = float(grid["x_min"])
    x_max = float(grid["x_max"])
    y_min = float(grid["y_min"])
    y_max = float(grid["y_max"])

    if resolution <= 0.0 or x_max <= x_min or y_max <= y_min:
        raise ValueError(f"Invalid {name} grid geometry in: {elevation_config}")

    # ROS ElevationGrid uses ceil((max - min) / resolution) cells.
    # IsaacLab GridPatternCfg produces round(size / resolution) + 1 rays.
    # Matching cell centers therefore requires:
    #   size = (ros_cell_count - 1) * resolution
    #   offset = midpoint of [min, max]
    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    return ElevationGridExample(
        name=name,
        resolution=resolution,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        width=width,
        height=height,
        isaac_size_x=(width - 1) * resolution,
        isaac_size_y=(height - 1) * resolution,
        isaac_offset_x=0.5 * (x_min + x_max),
        isaac_offset_y=0.5 * (y_min + y_max),
    )


def load_elevation_grid_examples(elevation_config: Path) -> Tuple[ElevationGridExample, ElevationGridExample | None]:
    if not elevation_config.exists():
        raise FileNotFoundError(f"Elevation config YAML does not exist: {elevation_config}")

    with elevation_config.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    params = data.get("elevation_mapping_node", {}).get("ros__parameters", {})
    height_map_grid = make_elevation_grid_example(
        "height_map",
        params.get("grid", {}),
        elevation_config,
    )
    local_grid_raw = (params.get("local_terrain_map", {}) or {}).get("grid")
    local_height_map_grid = None
    if isinstance(local_grid_raw, dict):
        local_height_map_grid = make_elevation_grid_example(
            "local_height_map",
            local_grid_raw,
            elevation_config,
        )

    return height_map_grid, local_height_map_grid


def resolve_camera_spec(model: str, stream: dict) -> CameraSpec:
    model_key = normalize_model_name(model)

    if model_key not in CAMERA_SPEC_DB:
        known = ", ".join(sorted(CAMERA_SPEC_DB.keys()))
        raise ValueError(f"Unknown camera model '{model}'. Known models: {known}")

    base = CAMERA_SPEC_DB[model_key]

    # YAML stream max_range가 있으면 모델 기본 max_depth보다 우선한다.
    max_depth = float(stream.get("max_range", base.max_depth))

    return CameraSpec(
        model=base.model,
        h_fov_deg=base.h_fov_deg,
        v_fov_deg=base.v_fov_deg,
        min_depth=base.min_depth,
        max_depth=max_depth,
    )


def resolve_cameras(
    bindings: List[CameraBinding],
    stream: dict,
    base_link: str,
    child_to_parent: Dict[str, dict],
    allow_non_fixed: bool = False,
) -> List[ResolvedCamera]:
    resolved = []

    for binding in bindings:
        spec = resolve_camera_spec(binding.model, stream)

        # FOV 판정은 optical_frame 기준이다.
        # 사용자가 F_camera_link 자체를 optical convention으로 정의했다면 그대로 사용 가능.
        T_base_optical = get_T_base_target(
            base_link=base_link,
            target_link=binding.optical_frame,
            child_to_parent=child_to_parent,
            allow_non_fixed=allow_non_fixed,
        )

        resolved.append(
            ResolvedCamera(
                role=binding.role,
                model=spec.model,
                mount_frame=binding.mount_frame,
                optical_frame=binding.optical_frame,
                T_base_optical=T_base_optical,
                spec=spec,
            )
        )

    return resolved


def generate_ray_caster_fov_py() -> str:
    return r'''from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import torch

from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors.ray_caster import RayCaster


@dataclass
class RayCasterFOVData:
    """Data container for camera-FOV-mask ray caster."""

    pos_w: torch.Tensor = None
    """Sensor origin position in world frame. Shape: (N, 3)."""

    quat_w: torch.Tensor = None
    """Sensor origin orientation in world frame. Shape: (N, 4), convention: wxyz."""

    ray_hits_w: torch.Tensor = None
    """Original RayCaster ray hit positions in world frame. Shape: (N, B, 3)."""

    raw_ray_hits_w: torch.Tensor = None
    """Same as original ray_hits_w. Kept explicitly for GT visualization/custom obs."""

    valid_mask: torch.Tensor = None
    """Combined camera FOV valid mask. Shape: (N, B). 1.0 = visible, 0.0 = invisible."""

    per_camera_valid_masks: dict = None
    """Per-camera valid masks. Dict[str, Tensor[N, B]]."""


class RayCasterFOV(RayCaster):
    """RayCaster that computes camera-FOV masks from actual ray hit points.

    Important:
        This class does NOT modify ray_hits_w.

    It is intended for:
        1. GT raw height scanner visualization
        2. camera-FOV-filtered point visualization
        3. later obs masking through a custom observation function
    """

    def __init__(self, *args, **kwargs):
        # IsaacLab SensorBase.__init__() may call set_debug_vis() immediately
        # when cfg.debug_vis=True. Therefore all debug visualizer attributes
        # must exist before super().__init__().
        self.gt_visualizer = None
        self.combined_valid_visualizer = None
        self.invalid_visualizer = None
        self.per_camera_visualizers = {}

        self._height_map_w = None
        self._height_map_h = None
        self._warned_zero_valid = False

        super().__init__(*args, **kwargs)

        # Replace parent RayCaster data container after parent initialization.
        # Parent update logic writes to fields with the same names.
        self._data = RayCasterFOVData()

    def __str__(self) -> str:
        valid_count = int(self._data.valid_mask[0].sum().item()) if self._data.valid_mask is not None else 0
        total_count = int(self._data.valid_mask.shape[1]) if self._data.valid_mask is not None else 0

        return (
            f"RayCasterFOV @ '{self.cfg.prim_path}': \n"
            f"\tview type            : {self._view.__class__}\n"
            f"\tupdate period (s)    : {self.cfg.update_period}\n"
            f"\tnumber of meshes     : {len(self.meshes)}\n"
            f"\tnumber of sensors    : {self._view.count}\n"
            f"\tnumber of rays/sensor: {self.num_rays}\n"
            f"\ttotal number of rays : {self.num_rays * self._view.count}\n"
            f"\tFOV valid rays       : {valid_count} / {total_count}\n"
        )

    @property
    def data(self) -> RayCasterFOVData:
        self._update_outdated_buffers()
        return self._data

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)

        if env_ids is None:
            env_ids = slice(None)

        if self._data.raw_ray_hits_w is not None:
            self._data.raw_ray_hits_w[env_ids] = 0.0

        if self._data.valid_mask is not None:
            self._data.valid_mask[env_ids] = 0.0

        if self._data.per_camera_valid_masks is not None:
            for mask in self._data.per_camera_valid_masks.values():
                mask[env_ids] = 0.0

    def _initialize_impl(self):
        super()._initialize_impl()

        self._height_map_w = int(round(self.cfg.pattern_cfg.size[0] / self.cfg.pattern_cfg.resolution) + 1)
        self._height_map_h = int(round(self.cfg.pattern_cfg.size[1] / self.cfg.pattern_cfg.resolution) + 1)

        expected_num_rays = self._height_map_w * self._height_map_h
        if expected_num_rays != self.num_rays:
            raise RuntimeError(
                f"RayCasterFOV grid mismatch: pattern_cfg gives "
                f"{self._height_map_h}x{self._height_map_w}={expected_num_rays} rays, "
                f"but RayCaster has num_rays={self.num_rays}. "
                f"This implementation assumes GridPatternCfg."
            )

        self._data.raw_ray_hits_w = torch.zeros(
            self._view.count,
            self.num_rays,
            3,
            dtype=torch.float32,
            device=self._device,
        )

        self._data.valid_mask = torch.zeros(
            self._view.count,
            self.num_rays,
            dtype=torch.float32,
            device=self._device,
        )

        self._data.per_camera_valid_masks = {}
        for cam in self._camera_configs():
            self._data.per_camera_valid_masks[cam["role"]] = torch.zeros(
                self._view.count,
                self.num_rays,
                dtype=torch.float32,
                device=self._device,
            )

    def _camera_configs(self):
        cameras = self.cfg.cameras
        if isinstance(cameras, dict):
            return (cameras,)
        return cameras

    @staticmethod
    def _quat_apply(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
        """Apply quaternion rotation to vector. Quaternion convention: w, x, y, z."""
        q_w = quat[..., 0:1]
        q_xyz = quat[..., 1:4]

        t = 2.0 * torch.cross(q_xyz, vec, dim=-1)
        return vec + q_w * t + torch.cross(q_xyz, t, dim=-1)

    @staticmethod
    def _quat_inv(quat: torch.Tensor) -> torch.Tensor:
        inv = quat.clone()
        inv[..., 1:4] *= -1.0
        return inv

    def _get_base_pose_from_sensor_pose(self, env_ids: Sequence[int]):
        """Return the base pose that RayCaster stored from cfg.prim_path.

        IsaacLab RayCaster applies cfg.offset.pos to ray_starts, not to data.pos_w.
        Since this sensor is attached to base_link, data.pos_w/data.quat_w already
        represent the base pose and must not subtract cfg.offset.pos again.
        """
        return self._data.pos_w[env_ids], self._data.quat_w[env_ids]

    def _world_hits_to_base(
        self,
        ray_hits_w: torch.Tensor,
        base_pos_w: torch.Tensor,
        base_quat_w: torch.Tensor,
    ) -> torch.Tensor:
        """Transform ray hit points from world frame to base-aligned frame."""
        rel_w = ray_hits_w - base_pos_w.unsqueeze(1)
        quat_inv = self._quat_inv(base_quat_w).unsqueeze(1)
        return self._quat_apply(quat_inv, rel_w)

    def _compute_fov_masks_from_hits(self, points_base: torch.Tensor):
        """Compute camera FOV masks from actual ray hit points.

        Args:
            points_base: Tensor shape [N, B, 3]

        Returns:
            combined_mask: Tensor shape [N, B], bool
            per_camera_masks: Dict[str, Tensor[N, B]], bool
        """
        num_envs, num_rays, _ = points_base.shape

        ones = torch.ones(
            num_envs,
            num_rays,
            1,
            device=self._device,
            dtype=torch.float32,
        )

        points_base_h = torch.cat([points_base, ones], dim=-1)

        combined = torch.zeros(
            num_envs,
            num_rays,
            device=self._device,
            dtype=torch.bool,
        )
        per_camera = {}

        for cam in self._camera_configs():
            T_base_optical = torch.tensor(
                cam["T_base_optical"],
                device=self._device,
                dtype=torch.float32,
            )

            T_optical_base = torch.linalg.inv(T_base_optical)

            # row-vector homogeneous transform
            points_optical = torch.matmul(points_base_h, T_optical_base.T)[..., :3]

            x = points_optical[..., 0]
            y = points_optical[..., 1]
            z = points_optical[..., 2]

            h_half = torch.deg2rad(
                torch.tensor(float(cam["h_fov_deg"]), device=self._device, dtype=torch.float32)
            ) * 0.5

            v_half = torch.deg2rad(
                torch.tensor(float(cam["v_fov_deg"]), device=self._device, dtype=torch.float32)
            ) * 0.5

            valid_depth = (z > float(cam["min_depth"])) & (z < float(cam["max_depth"]))
            valid_h = torch.abs(torch.atan2(x, z)) < h_half
            valid_v = torch.abs(torch.atan2(y, z)) < v_half

            mask = valid_depth & valid_h & valid_v

            per_camera[cam["role"]] = mask
            combined |= mask

        return combined, per_camera

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        super()._update_buffers_impl(env_ids)

        # Keep original RayCaster result as GT.
        raw_hits = self._data.ray_hits_w[env_ids].clone()
        self._data.raw_ray_hits_w[env_ids] = raw_hits

        # Compute FOV mask using actual 3D hit points.
        base_pos_w, base_quat_w = self._get_base_pose_from_sensor_pose(env_ids)
        points_base = self._world_hits_to_base(raw_hits, base_pos_w, base_quat_w)

        combined_mask, per_camera_masks = self._compute_fov_masks_from_hits(points_base)

        self._data.valid_mask[env_ids] = combined_mask.to(dtype=torch.float32)

        for role, mask in per_camera_masks.items():
            self._data.per_camera_valid_masks[role][env_ids] = mask.to(dtype=torch.float32)

        if bool(self.cfg.debug_print_valid_counts):
            counts = combined_mask.sum(dim=1)
            if torch.any(counts > 0) or not self._warned_zero_valid:
                per_role_counts = {
                    role: mask.sum(dim=1).detach().cpu().tolist() for role, mask in per_camera_masks.items()
                }
                print(
                    "[RayCasterFOV] valid_counts="
                    f"{counts.detach().cpu().tolist()} per_camera={per_role_counts}"
                )
                self._warned_zero_valid = bool(torch.all(counts == 0))

        # IMPORTANT:
        # Do not modify self._data.ray_hits_w.
        # Observation masking should use data.valid_mask.

    def _set_debug_vis_impl(self, debug_vis: bool):
        # Extra guard for early SensorBase initialization.
        if not hasattr(self, "gt_visualizer"):
            self.gt_visualizer = None
        if not hasattr(self, "combined_valid_visualizer"):
            self.combined_valid_visualizer = None
        if not hasattr(self, "invalid_visualizer"):
            self.invalid_visualizer = None
        if not hasattr(self, "per_camera_visualizers"):
            self.per_camera_visualizers = {}

        if debug_vis:
            if self.gt_visualizer is None:
                self.gt_visualizer = VisualizationMarkers(self.cfg.gt_visualizer_cfg)
            if self.combined_valid_visualizer is None:
                self.combined_valid_visualizer = VisualizationMarkers(self.cfg.combined_valid_visualizer_cfg)
            if self.invalid_visualizer is None:
                self.invalid_visualizer = VisualizationMarkers(self.cfg.invalid_visualizer_cfg)

            if not self.per_camera_visualizers:
                for role, cfg in self.cfg.per_camera_visualizer_cfgs.items():
                    self.per_camera_visualizers[role] = VisualizationMarkers(cfg)

            self.gt_visualizer.set_visibility(bool(self.cfg.debug_show_gt))
            self.combined_valid_visualizer.set_visibility(bool(self.cfg.debug_show_combined_valid))
            self.invalid_visualizer.set_visibility(bool(self.cfg.debug_show_invalid))

            for role, visualizer in self.per_camera_visualizers.items():
                visualizer.set_visibility(bool(self.cfg.debug_show_per_camera))

        else:
            if self.gt_visualizer is not None:
                self.gt_visualizer.set_visibility(False)
            if self.combined_valid_visualizer is not None:
                self.combined_valid_visualizer.set_visibility(False)
            if self.invalid_visualizer is not None:
                self.invalid_visualizer.set_visibility(False)
            for visualizer in self.per_camera_visualizers.values():
                visualizer.set_visibility(False)

    @staticmethod
    def _visualize_or_hide(visualizer, points: torch.Tensor, visible: bool):
        if visualizer is None:
            return
        if not visible or points.numel() == 0:
            visualizer.set_visibility(False)
            return
        visualizer.set_visibility(True)
        visualizer.visualize(points.reshape(-1, 3))

    def _with_debug_z_offset(self, points: torch.Tensor, z_offset: float) -> torch.Tensor:
        if points.numel() == 0 or z_offset == 0.0:
            return points
        offset_points = points.clone()
        offset_points[..., 2] += float(z_offset)
        return offset_points

    def _debug_vis_callback(self, event):
        if self._data.ray_hits_w is None or self._data.valid_mask is None:
            return

        ray_hits_w = self._data.ray_hits_w

        self._visualize_or_hide(
            self.gt_visualizer,
            ray_hits_w,
            bool(self.cfg.debug_show_gt),
        )

        valid = self._data.valid_mask.bool()

        self._visualize_or_hide(
            self.combined_valid_visualizer,
            self._with_debug_z_offset(ray_hits_w[valid], self.cfg.debug_valid_z_offset),
            bool(self.cfg.debug_show_combined_valid),
        )

        if bool(self.cfg.debug_show_invalid):
            invalid = ~valid
            self._visualize_or_hide(
                self.invalid_visualizer,
                ray_hits_w[invalid],
                True,
            )
        elif self.invalid_visualizer is not None:
            self.invalid_visualizer.set_visibility(False)

        if bool(self.cfg.debug_show_per_camera):
            for role, mask_float in self._data.per_camera_valid_masks.items():
                if role not in self.per_camera_visualizers:
                    continue

                mask = mask_float.bool()
                self._visualize_or_hide(
                    self.per_camera_visualizers[role],
                    self._with_debug_z_offset(ray_hits_w[mask], self.cfg.debug_valid_z_offset),
                    True,
                )
        else:
            for visualizer in self.per_camera_visualizers.values():
                visualizer.set_visibility(False)
'''

def grid_pattern_literal(grid: ElevationGridExample) -> str:
    return (
        "{"
        f'"resolution": {grid.resolution:g}, '
        f'"size": ({grid.isaac_size_x:g}, {grid.isaac_size_y:g}), '
        f'"offset": ({grid.isaac_offset_x:g}, {grid.isaac_offset_y:g}), '
        f'"ros_width": {grid.width}, '
        f'"ros_height": {grid.height}'
        "}"
    )


def generate_ray_caster_fov_cfg_py(
    cameras: List[ResolvedCamera],
    module_name: str,
    height_map_grid: ElevationGridExample,
    local_height_map_grid: ElevationGridExample | None,
) -> str:
    camera_entries = []
    per_camera_marker_entries = []

    # 카메라별 시각화 색상 팔레트
    color_palette = [
        (0.0, 0.4, 1.0),   # blue
        (1.0, 0.7, 0.0),   # yellow/orange
        (1.0, 0.0, 1.0),   # magenta
        (0.0, 1.0, 1.0),   # cyan
        (0.5, 1.0, 0.0),   # lime
    ]

    for idx, cam in enumerate(cameras):
        T = cam.T_base_optical
        T_literal = (
            "("
            + ", ".join(
                "(" + ", ".join(f"{float(v):.10f}" for v in row) + ")"
                for row in T
            )
            + ")"
        )

        camera_entries.append(
            f'''    {{
        "role": "{cam.role}",
        "model": "{cam.model}",
        "mount_frame": "{cam.mount_frame}",
        "optical_frame": "{cam.optical_frame}",
        "h_fov_deg": {cam.spec.h_fov_deg:.10f},
        "v_fov_deg": {cam.spec.v_fov_deg:.10f},
        "min_depth": {cam.spec.min_depth:.10f},
        "max_depth": {cam.spec.max_depth:.10f},
        "T_base_optical": {T_literal},
    }}'''
        )

        color = color_palette[idx % len(color_palette)]
        per_camera_marker_entries.append(
            f'''    "{cam.role}": VisualizationMarkersCfg(
        prim_path="/Visuals/RayCasterFOV/{cam.role}",
        markers={{
            "hit": sim_utils.SphereCfg(
                radius=0.028,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color={color}),
            ),
        }},
    )'''
        )

    # Always emit tuple syntax. With a single camera, `({...})` becomes a dict,
    # while `({...},)` remains a one-item tuple.
    camera_block = ",\n".join(camera_entries) + ","
    per_camera_marker_block = ",\n".join(per_camera_marker_entries)
    local_pattern = (
        grid_pattern_literal(local_height_map_grid)
        if local_height_map_grid is not None
        else "None"
    )

    return f'''from isaaclab.utils import configclass
from isaaclab.markers import VisualizationMarkersCfg
import isaaclab.sim as sim_utils
from isaaclab.sensors import RayCasterCfg

from .{module_name} import RayCasterFOV


GT_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RayCasterFOV/GT",
    markers={{
        "hit": sim_utils.SphereCfg(
            radius=0.008,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.12, 0.12, 0.12)),
        ),
    }},
)

COMBINED_VALID_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RayCasterFOV/CombinedValid",
    markers={{
        "hit": sim_utils.SphereCfg(
            radius=0.035,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
        ),
    }},
)

INVALID_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RayCasterFOV/Invalid",
    markers={{
        "hit": sim_utils.SphereCfg(
            radius=0.016,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
    }},
)


PER_CAMERA_MARKER_CFGS = {{
{per_camera_marker_block}
}}


CAMERA_FOV_DATA = (
{camera_block}
)


HEIGHT_MAP_PATTERN = {grid_pattern_literal(height_map_grid)}
LOCAL_HEIGHT_MAP_PATTERN = {local_pattern}


@configclass
class RayCasterFOVCfg(RayCasterCfg):
    """RayCaster config with URDF+camera-FOV mask.

    This sensor does not modify ray_hits_w.
    It computes:
      - raw_ray_hits_w
      - valid_mask
      - per_camera_valid_masks

    Debug visualization:
      - GT gray points: original height scanner grid
      - combined valid green points: any camera can see
      - per-camera colored points: each camera's FOV result
      - invalid red points: optional, usually noisy because it shows the full remaining rectangle
    """

    class_type: type = RayCasterFOV

    gt_visualizer_cfg: VisualizationMarkersCfg = GT_MARKER_CFG
    combined_valid_visualizer_cfg: VisualizationMarkersCfg = COMBINED_VALID_MARKER_CFG
    invalid_visualizer_cfg: VisualizationMarkersCfg = INVALID_MARKER_CFG
    per_camera_visualizer_cfgs: dict = PER_CAMERA_MARKER_CFGS

    cameras: tuple = CAMERA_FOV_DATA

    # 전체 기존 height scanner GT를 회색으로 표시.
    debug_show_gt: bool = True

    # 모든 카메라를 합친 valid 영역을 초록색으로 표시.
    # per-camera와 겹쳐 보이면 False로 꺼도 됨.
    debug_show_combined_valid: bool = True

    # 카메라별 FOV 영역을 색상별로 표시.
    debug_show_per_camera: bool = False

    # FOV 밖 점을 빨간색으로 표시.
    # 전체 사각형이 많이 보이므로 필요할 때만 True.
    debug_show_invalid: bool = False

    # FOV 점을 GT 점보다 살짝 위에 그려 viewport에서 가려지지 않게 한다.
    debug_valid_z_offset: float = 0.03

    # True면 valid ray 개수를 콘솔에 출력해서 FOV 계산 상태를 확인한다.
    debug_print_valid_counts: bool = False
'''

def parse_args():
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Generate IsaacLab RayCasterFOV sensor code from URDF and camera binding YAML."
    )

    parser.add_argument(
        "--mapping-file",
        default=str(root / "resources" / "config" / "autonomy.yaml"),
        help="Camera binding YAML path.",
    )
    parser.add_argument(
        "--urdf",
        default=str(root / "resources" / "urdf" / "f16.urdf"),
        help="Robot URDF path.",
    )
    parser.add_argument(
        "--elevation-config",
        default=str(root / "resources" / "config" / "autonomy.yaml"),
        help="Elevation mapping YAML path used to print a matching IsaacLab GridPatternCfg example.",
    )
    parser.add_argument(
        "--base-link",
        default="base_link",
        help="Base link name in URDF.",
    )

    parser.add_argument(
        "--output-dir",
        default=str(root / "resources" / "sensors"),
        help="Directory where ray_caster_fov.py and ray_caster_fov_cfg.py will be written.",
    )

    parser.add_argument(
        "--module-name",
        default="ray_caster_fov",
        help="Base module name. Default generates ray_caster_fov.py and ray_caster_fov_cfg.py.",
    )

    parser.add_argument(
        "--allow-non-fixed",
        action="store_true",
        help="Allow non-fixed joints in URDF chain by using URDF origin only.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    mapping_file = Path(args.mapping_file).expanduser().resolve()
    urdf_path = Path(args.urdf).expanduser().resolve()
    elevation_config = Path(args.elevation_config).expanduser().resolve()
    base_link = normalize_frame_name(args.base_link)

    bindings, stream = load_mapping(mapping_file)
    height_map_grid, local_height_map_grid = load_elevation_grid_examples(elevation_config)
    link_names, child_to_parent, _ = load_urdf_tree(urdf_path)

    if base_link not in link_names:
        print(
            f"[WARN] base_link '{base_link}' was not found as a <link> name in URDF. "
            f"Continuing anyway because it may be namespaced differently.",
            file=sys.stderr,
        )

    resolved_cameras = resolve_cameras(
        bindings=bindings,
        stream=stream,
        base_link=base_link,
        child_to_parent=child_to_parent,
        allow_non_fixed=args.allow_non_fixed,
    )

    if not resolved_cameras:
        raise RuntimeError("No cameras were resolved from YAML + URDF.")

    out_dir = Path(args.output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    module_name = args.module_name
    sensor_path = out_dir / f"{module_name}.py"
    cfg_path = out_dir / f"{module_name}_cfg.py"
    init_path = out_dir / "__init__.py"

    header = textwrap.dedent(
        f"""\
        # Auto-generated by generate_height_filter_from_urdf.py
        # Source mapping: {mapping_file}
        # Source URDF: {urdf_path}
        #
        """
    )

    ray_caster_code = generate_ray_caster_fov_py()
    ray_caster_cfg_code = generate_ray_caster_fov_cfg_py(
        cameras=resolved_cameras,
        module_name=module_name,
        height_map_grid=height_map_grid,
        local_height_map_grid=local_height_map_grid,
    )

    sensor_path.write_text(header + ray_caster_code, encoding="utf-8")
    cfg_path.write_text(header + ray_caster_cfg_code, encoding="utf-8")

    if not init_path.exists():
        init_path.write_text("", encoding="utf-8")

    print(f"Wrote: {sensor_path}")
    print(f"Wrote: {cfg_path}")
    print(f"Ensured package init: {init_path}")
    print(f"Resolved {len(resolved_cameras)} camera(s):")

    for cam in resolved_cameras:
        print(
            f"  role={cam.role} model={cam.model} "
            f"mount={cam.mount_frame} optical={cam.optical_frame} "
            f"max_depth={cam.spec.max_depth}"
        )

    print("")
    print("Use in IsaacLab config:")
    print(f"  from <your_package>.resources.sensors.{module_name}_cfg import RayCasterFOVCfg")
    print("")
    print("  # Matched to autonomy.yaml height_map grid:")
    print(
        f"  #   ROS cells: width={height_map_grid.width}, height={height_map_grid.height}, "
        f"resolution={height_map_grid.resolution:g}"
    )
    print("  #   ROS ElevationGrid uses ceil((max - min) / resolution).")
    print("  #   IsaacLab GridPatternCfg uses round(size / resolution) + 1 rays.")
    print("")
    print("  height_scanner = RayCasterFOVCfg(")
    print('      prim_path="{ENV_REGEX_NS}/Robot/base_link",')
    print(
        "      offset=RayCasterFOVCfg.OffsetCfg("
        f"pos=({height_map_grid.isaac_offset_x:g}, {height_map_grid.isaac_offset_y:g}, 20.0)"
        "),"
    )
    print('      ray_alignment="yaw",')
    print(
        "      pattern_cfg=patterns.GridPatternCfg("
        f"resolution={height_map_grid.resolution:g}, "
        f"size=[{height_map_grid.isaac_size_x:g}, {height_map_grid.isaac_size_y:g}]"
        "),"
    )
    print("      debug_vis=True,")
    print('      mesh_prim_paths=["/World/ground"],')
    print("      debug_show_invalid=False,")
    print("  )")
    if local_height_map_grid is not None:
        print("")
        print("  # Add separately in adas/fsd modes to match local_terrain_map.grid:")
        print("  local_height_map = RayCasterFOVCfg(")
        print('      prim_path="{ENV_REGEX_NS}/Robot/base_link",')
        print(
            "      offset=RayCasterFOVCfg.OffsetCfg("
            f"pos=({local_height_map_grid.isaac_offset_x:g}, {local_height_map_grid.isaac_offset_y:g}, 20.0)"
            "),"
        )
        print('      ray_alignment="yaw",')
        print(
            "      pattern_cfg=patterns.GridPatternCfg("
            f"resolution={local_height_map_grid.resolution:g}, "
            f"size=[{local_height_map_grid.isaac_size_x:g}, {local_height_map_grid.isaac_size_y:g}]"
            "),"
        )
        print("      debug_vis=True,")
        print('      mesh_prim_paths=["/World/ground"],')
        print("      debug_show_invalid=False,")
        print("  )")
    print("")
    print("Use in an IsaacLab observation file:")
    print("")
    print("import torch")
    print("from isaaclab.envs import ManagerBasedEnv")
    print("from isaaclab.managers import SceneEntityCfg")
    print("")
    print("def masked_height_scan(")
    print("    env: ManagerBasedEnv,")
    print("    sensor_cfg: SceneEntityCfg,")
    print("    offset: float = 0.5,")
    print("    base_height: float = 0.5,")
    print(") -> torch.Tensor:")
    print('    \"\"\"Camera-FOV masked height scan for sim-to-real matching.')
    print("")
    print("    Inside camera FOV, this returns the normal height scan:")
    print("        sensor_height - hit_point_z - offset")
    print("")
    print("    Outside camera FOV, it returns base_height - offset, which mimics")
    print("    unobserved cells being filled with the robot base-height prior.")
    print('    \"\"\"')
    print("    sensor = env.scene.sensors[sensor_cfg.name]")
    print("    height = sensor.data.pos_w[:, 2].unsqueeze(1) - sensor.data.ray_hits_w[..., 2] - offset")
    print("    fill_value = torch.full_like(height, float(base_height) - float(offset))")
    print("    valid_mask = sensor.data.valid_mask.bool()")
    print("    return torch.where(valid_mask, height, fill_value)")


if __name__ == "__main__":
    main()
