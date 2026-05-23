#!/usr/bin/env python3
"""Generate a RealSense camera mapping YAML skeleton from URDF camera links."""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


DEFAULT_STREAM = {
    "depth_width": 640,
    "depth_height": 480,
    "depth_fps": 60,
    "max_range": 2.5,
    "fallback_profiles": [
        {"depth_width": 640, "depth_height": 480, "depth_fps": 30},
        {"depth_width": 848, "depth_height": 480, "depth_fps": 60},
        {"depth_width": 640, "depth_height": 360, "depth_fps": 60},
        {"depth_width": 424, "depth_height": 240, "depth_fps": 60},
    ],
}


DEFAULT_MERGE_PARAMETERS = {
    "target_frame": "4w4l/base_link",
    "publish_rate_hz": 20.0,
    "max_cloud_age_sec": 0.0,
    "depth_filter": {
        "min_range": 0.05,
        "max_range": 2.5,
        "pixel_stride": 2,
    },
}


DEFAULT_TOPIC_PREFIX = "/robot_4w4l"


@dataclass(frozen=True)
class CameraLink:
    link_name: str
    role: str
    camera_name: str
    optical_frame: str


def package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def normalize_name(name: str) -> str:
    return str(name).strip().lstrip("/")


def safe_topic_name(name: str) -> str:
    text = normalize_name(name).lower()
    text = re.sub(r"(_)?camera(_)?link$", "", text)
    text = re.sub(r"(_)?link$", "", text)
    text = text.strip("_/")
    text = re.sub(r"[^a-z0-9_]+", "_", text)
    text = re.sub(r"_+", "_", text).strip("_")
    return text or "camera"


def realsense_optical_frame(camera_name: str) -> str:
    return f"{normalize_name(camera_name)}_depth_optical_frame"


def infer_role(link_name: str, used_roles: set[str], index: int) -> str:
    normalized = normalize_name(link_name)
    lowered = normalized.lower()

    candidates: list[str] = []
    if lowered in ("f_camera_link", "front_camera_link") or lowered.startswith(("f_camera", "front_camera")):
        candidates.append("front")
    if lowered in ("r_camera_link", "rear_camera_link") or lowered.startswith(("r_camera", "rear_camera", "back_camera")):
        candidates.append("rear")
    if lowered in ("a_camera_link", "adas_camera_link") or lowered.startswith(("a_camera", "adas_camera")):
        candidates.append("adas")
    if lowered.startswith(("l_camera", "left_camera")):
        candidates.append("left")
    if lowered.startswith(("right_camera", "rt_camera")):
        candidates.append("right")

    if not candidates:
        topic_base = safe_topic_name(normalized)
        if topic_base not in {"camera", "camera_link"}:
            candidates.append(topic_base)

    candidates.append(f"camera_{index + 1}")

    for candidate in candidates:
        role = candidate
        suffix = 2
        while role in used_roles:
            role = f"{candidate}_{suffix}"
            suffix += 1
        if role not in used_roles:
            used_roles.add(role)
            return role

    raise RuntimeError("Unable to infer a unique camera role")


def is_camera_link_candidate(link_name: str) -> bool:
    lowered = normalize_name(link_name).lower()
    if "camera" not in lowered:
        return False
    if not lowered.endswith("link"):
        return False
    ignored_tokens = ("optical", "depth", "color", "infra", "ir")
    return not any(token in lowered for token in ignored_tokens)


def parse_urdf(urdf_path: Path) -> tuple[set[str], dict[str, list[str]]]:
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF does not exist: {urdf_path}")

    root = ET.parse(str(urdf_path)).getroot()

    links = {
        normalize_name(link.get("name", ""))
        for link in root.findall("link")
        if link.get("name")
    }

    children_by_parent: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent = normalize_name(parent_elem.get("link", ""))
        child = normalize_name(child_elem.get("link", ""))
        if parent and child:
            children_by_parent.setdefault(parent, []).append(child)

    return links, children_by_parent


def choose_optical_frame(link_name: str, children_by_parent: dict[str, list[str]]) -> str:
    children = children_by_parent.get(link_name, [])
    for child in children:
        if "optical" in child.lower():
            return child
    return link_name


def find_camera_links(urdf_path: Path) -> list[CameraLink]:
    links, children_by_parent = parse_urdf(urdf_path)
    candidates = sorted(link for link in links if is_camera_link_candidate(link))

    used_roles: set[str] = set()
    camera_links: list[CameraLink] = []
    for index, link_name in enumerate(candidates):
        role = infer_role(link_name, used_roles, index)
        camera_name = f"{role}_camera" if not role.endswith("_camera") else role
        camera_links.append(
            CameraLink(
                link_name=link_name,
                role=role,
                camera_name=camera_name,
                optical_frame=choose_optical_frame(link_name, children_by_parent),
            )
        )

    return camera_links


def quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def render_mapping(camera_links: list[CameraLink], urdf_path: Path) -> str:
    lines = [
        "# Camera role, USB port, topic, and point cloud merge map.",
        "# Auto-generated from URDF camera links.",
        "# Fill real usb_port_id values before running the hardware mapper.",
        "",
        "operation_modes:",
        "  drive:",
        "    camera_roles: [front, rear]",
        "    require_roles: []",
        "",
        "  adas:",
        "    camera_roles: [front, rear, adas]",
        "    require_roles: [adas]",
        "",
        "  fsd:",
        "    camera_roles: [front, rear, adas]",
        "    require_roles: [adas]",
        "",
        "  tracking:",
        "    camera_roles: [front, rear, adas]",
        "    require_roles: [adas]",
        "",
        "camera_bindings:",
        "  simulation:",
    ]

    if not camera_links:
        lines.append("    []")
    else:
        for camera in camera_links:
            topic_base = safe_topic_name(camera.camera_name)
            lines.extend(
                [
                    f"    - role: {camera.role}",
                    "      enabled: true",
                    "      model: d435",
                    f"      camera_name: {camera.camera_name}",
                    f"      depth_topic: {DEFAULT_TOPIC_PREFIX}/{topic_base}/depth/image_rect_raw",
                    f"      camera_info_topic: {DEFAULT_TOPIC_PREFIX}/{topic_base}/depth/camera_info",
                    f"      mount_frame: {camera.link_name}",
                    f"      optical_frame: {camera.optical_frame}",
                    "",
                ]
            )

    lines.extend(["  real:"])

    if not camera_links:
        lines.append("    []")
    else:
        for camera in camera_links:
            topic_base = safe_topic_name(camera.camera_name)
            lines.extend(
                [
                    f"    - role: {camera.role}",
                    "      enabled: true",
                    '      usb_port_id: ""',
                    "      model: d435",
                    f"      camera_name: {camera.camera_name}",
                    f"      depth_topic: /{topic_base}/depth/image_rect_raw",
                    f"      camera_info_topic: /{topic_base}/depth/camera_info",
                    f"      mount_frame: {camera.link_name}",
                    f"      optical_frame: {camera.optical_frame}",
                    "",
                ]
            )

    lines.extend(
        [
            "stream:",
            f"  depth_width: {DEFAULT_STREAM['depth_width']}",
            f"  depth_height: {DEFAULT_STREAM['depth_height']}",
            f"  depth_fps: {DEFAULT_STREAM['depth_fps']}",
            f"  max_range: {DEFAULT_STREAM['max_range']}",
            "  fallback_profiles:",
        ]
    )

    for profile in DEFAULT_STREAM["fallback_profiles"]:
        lines.append(
            "    - "
            f"{{depth_width: {profile['depth_width']}, "
            f"depth_height: {profile['depth_height']}, "
            f"depth_fps: {profile['depth_fps']}}}"
        )

    lines.extend(
        [
            "",
            "robot_state_publisher:",
            "  ros__parameters:",
            "    enabled: true",
            f"    urdf_path: src/autonomy/resources/urdf/{urdf_path.name}",
            "",
            "pointcloud_merge_node:",
            "  ros__parameters:",
            f"    target_frame: {DEFAULT_MERGE_PARAMETERS['target_frame']}",
            f"    publish_rate_hz: {DEFAULT_MERGE_PARAMETERS['publish_rate_hz']}",
            f"    max_cloud_age_sec: {DEFAULT_MERGE_PARAMETERS['max_cloud_age_sec']}",
            "",
            "    depth_filter:",
            f"      min_range: {DEFAULT_MERGE_PARAMETERS['depth_filter']['min_range']}",
            f"      max_range: {DEFAULT_MERGE_PARAMETERS['depth_filter']['max_range']}",
            f"      pixel_stride: {DEFAULT_MERGE_PARAMETERS['depth_filter']['pixel_stride']}",
        ]
    )

    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    root = package_root()
    parser = argparse.ArgumentParser(
        description="Generate camera binding YAML from URDF camera links."
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=root / "resources" / "urdf" / "f16.urdf",
        help="Path to the robot URDF.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "resources" / "config" / "camera_bindings.generated.yaml",
        help="Output YAML path.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the generated YAML instead of writing it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_links = find_camera_links(args.urdf)
    text = render_mapping(camera_links, args.urdf)

    if args.dry_run:
        print(text, end="")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
        print(f"Wrote {args.output}")

    print(f"Found {len(camera_links)} camera link(s):", file=sys.stderr)
    for camera in camera_links:
        print(
            f"  role={camera.role} link={camera.link_name} optical={camera.optical_frame}",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
