# autonomy

ROS2 integration layer for merging multiple RealSense point clouds into `base_link`
and producing a robot-centric elevation image.

## Structure

- `realsense_usb_mapper.py`: hardware-only node that maps RealSense devices
  by USB physical port and publishes per-camera depth images plus `CameraInfo`.
- `PointCloudMergerNode`: subscribes to each camera depth image and intrinsics,
  publishes static TF from the URDF, back-projects depth maps into 3D, transforms
  points into `target_frame`, and publishes `~/merged_points`.
- `ElevationMappingNode`: subscribes to the merged point cloud and publishes
  `~/elevation_image` (`sensor_msgs/Image`, `32FC1`), the ROS2
  `~/masked_height_scan`, and a DDS `core_dds::HeightMap` sample on
  `height_map`.
- `core/msg/CommandFilter`: publishes movement-axis allow/deny decisions from the
  latest height map on `/command_filter`.
- `HeightMapFrame`: internal model used to keep ROS2 and DDS output contracts
  separate.
- `ElevationMapBackend`: algorithm boundary for elevation conversion.
- `MinZElevationBackend`: simple starter backend that stores the minimum z value
  per grid cell.

Replace `MinZElevationBackend` with a backend wrapping the current elevation map
algorithm when the algorithm is refactored to accept an already merged point cloud.

## Expected TF

The active URDF already has fixed camera links:

- `F_camera_link -> base_link`
- `R_camera_link -> base_link`

Make sure the frame in each incoming RealSense point cloud is connected to these
links. If the RealSense driver publishes an optical frame such as
`front_camera_depth_optical_frame`, add the optical frame transform in URDF or with
`static_transform_publisher`.

## Build

Use `scripts/build.sh` as the single entrypoint for both the simulation machine
and Jetson hardware.

Simulation/development machine:

```bash
mkdir -p ~/ros2_ws/src
ln -s /home/sunrise/SH/height_cli_lib_new/ros2/autonomy ~/ros2_ws/src/autonomy
~/ros2_ws/src/autonomy/scripts/build.sh sim
source ~/ros2_ws/install/setup.bash
```

Jetson hardware setup and build:

```bash
mkdir -p ~/ros2_ws/src
ln -s /path/to/autonomy ~/ros2_ws/src/autonomy
~/ros2_ws/src/autonomy/scripts/build.sh jetson
source ~/ros2_ws/install/setup.bash
```

After the first Jetson setup, rebuild only this stack with:

```bash
~/ros2_ws/src/autonomy/scripts/build.sh jetson \
  --skip-ros --skip-librealsense --skip-realsense-ros
```

## Run

The autonomy stack is launched from a single launch file and a single parameter
file, `resources/config/autonomy.yaml`. On startup the stack stays in `IDLE`; change modes
at runtime with `/autonomy_manager/set_mode`.

Simulation:

```bash
ros2 launch autonomy autonomy.launch.py simulation:=true
```

Hardware:

```bash
ros2 launch autonomy autonomy.launch.py simulation:=false
```

Keep USB port bindings, camera topic bindings, merge settings, elevation tuning,
SLAM, AI, planner, and manager parameters in `resources/config/autonomy.yaml`. For
hardware, first identify the board port ids:

```bash
sudo apt install python3-opencv python3-yaml
python3 -m pip install pyrealsense2
ros2 run autonomy show_realsense_usb_ports.py
```

Each connected RealSense color image is displayed with its `usb_port_id`, serial,
and model overlaid. Plug a camera into each board port, note the reported
`usb_port_id`, then edit `resources/config/autonomy.yaml`. After that, any
camera plugged into that physical port is mapped to the configured role while
`enabled` still controls whether the role is used.

The hardware mapper resolves configured USB ports to the currently connected
device serials, publishes per-camera depth images plus `CameraInfo` at the
configured depth FPS, and publishes the resolved bindings as JSON on
`~/camera_bindings`.

Change mode at runtime:

```bash
ros2 service call /autonomy_manager/set_mode autonomy/srv/SetAutonomyMode \
"{operation_mode: 'ADAS', speed_limit: 0.8, enable_ai: true, segmentation: false}"
```

ESTOP is derived from `/robot_report`. `physical_estop` or `comm_estop` forces
the effective autonomy mode to `IDLE`; clearing both restores the previously
requested mode.

Monitor the manager status:

```bash
ros2 topic echo /autonomy_manager/status
```

## API Reference

This section describes the runtime API from a user point of view: what to
publish into this package, what to subscribe to, and how to interpret the data.
Topic names below are the defaults from `resources/config/autonomy.yaml`.

### Main data outputs

| Topic | Direction | Type | Producer | What you get |
| --- | --- | --- | --- | --- |
| `/elevation_mapping_node/masked_height_scan` | output | `autonomy/msg/MaskedHeightScan` | `elevation_mapping_node` | Robot-centric height grid with metadata and a validity mask. |
| `/command_filter` | output | `core/msg/CommandFilter` | `elevation_mapping_node` | Whether linear x/y movement is currently allowed. |
| DDS `height_map` | output | `core_dds::HeightMap` | `elevation_mapping_node` | Compact DDS height-map sample: `sequence<float> data`. |
| `/elevation_mapping_node/elevation_image` | output | `sensor_msgs/msg/Image` | `elevation_mapping_node` | Debug elevation image, encoding `32FC1`. |
| `/elevation_mapping_node/elevation_points` | output | `sensor_msgs/msg/PointCloud2` | `elevation_mapping_node` | Debug point cloud generated from the elevation grid. |
| `/autonomy_manager/status` | output | `autonomy/msg/AutonomyState` | `autonomy_manager_node` | Current autonomy mode, ESTOP/error state, robot report fields, node health. |
| `/autonomy_manager/state` | output | `autonomy/msg/AutonomyState` | `autonomy_manager_node` | Same state contract as `/autonomy_manager/status`; useful for direct manager state consumers. |

### Main inputs

| Topic or service | Direction | Type | Consumer | Purpose |
| --- | --- | --- | --- | --- |
| `/pointcloud_merge_node/merged_points` | input | `sensor_msgs/msg/PointCloud2` | `elevation_mapping_node` | Merged depth-camera point cloud in the robot frame. |
| `/robot_report` | input | `core/msg/RobotReport` | `autonomy_manager_node` | Robot state, ESTOP, communication fault, and error reason from the robot interface. |
| `/autonomy_manager/set_mode` | service | `autonomy/srv/SetAutonomyMode` | `autonomy_manager_node` | Request autonomy mode, speed limit, AI enable, and segmentation enable. |
| `/autonomy_manager/status` | input | `autonomy/msg/AutonomyState` | autonomy child nodes | Tells child nodes whether their mode is active. |

### Receiving the height scan in ROS 2

Subscribe to `/elevation_mapping_node/masked_height_scan` when you need the full
ROS 2 height-map contract:

```bash
ros2 topic echo /elevation_mapping_node/masked_height_scan
```

Message type:

```text
autonomy/msg/MaskedHeightScan
```

Field meaning:

| Field | Meaning |
| --- | --- |
| `header` | Source timestamp and frame. |
| `width`, `height` | Grid dimensions. `data.size()` is `width * height`. |
| `resolution` | Cell size in meters. |
| `x_min`, `x_max`, `y_min`, `y_max` | Grid bounds in the robot frame. |
| `height_scan_offset` | Offset used to match the IsaacLab-style height scan. |
| `base_height` | Nominal robot base height used for unobserved cells. |
| `fill_value` | Value used for unobserved cells. |
| `data` | Row-major float height values. Index is `row * width + col`. |
| `valid_mask` | Row-major validity flags. `1` means observed; `0` means filled/unobserved. |

The height value follows:

```text
observed   = -point_z_in_base_frame - height_scan_offset
unobserved = base_height - height_scan_offset
```

For a grid coordinate:

```text
x = x_min + col * resolution
y = y_min + row * resolution
index = row * width + col
height = data[index]
valid = valid_mask[index] != 0
```

### Receiving the command filter

Subscribe to `/command_filter` when you only need a movement allow/deny signal:

```bash
ros2 topic echo /command_filter
```

Message type:

```text
core/msg/CommandFilter
```

Fields:

| Field | Meaning |
| --- | --- |
| `allow_linear_vel_forward_x` | `true` means positive x velocity is allowed. |
| `allow_linear_vel_backward_x` | `true` means negative x velocity is allowed. |
| `allow_linear_vel_forward_y` | `true` means positive y velocity is allowed. |
| `allow_linear_vel_backward_y` | `true` means negative y velocity is allowed. |

If no height map has arrived yet, all fields default to `true`. A field becomes
`false` when the latest height map contains an obstacle at least the configured
`command_filter.obstacle_height_threshold` above `command_filter.obstacle_floor_z`
inside that movement corridor.

### Receiving the DDS height map

Use DDS topic `height_map` when the consumer is outside ROS 2 or talks directly
to DDS. The default DDS contract is:

| Item | Value |
| --- | --- |
| Domain id | `0` |
| Topic | `height_map` |
| Type | `core_dds::HeightMap` |
| QoS | keep last depth `1`, reliable |
| Payload | `data`, a `sequence<float>` |

IDL:

```idl
module core_dds {
  struct HeightMap {
    sequence<float> data;
  };
};
```

The DDS payload intentionally contains only the float array. Use the ROS 2
`MaskedHeightScan` topic when the consumer also needs `width`, `height`,
`resolution`, bounds, or `valid_mask`. The sample length follows the configured
grid size, so it is not hard-coded to `144`.

### Robot report input expected from core

Publish `/robot_report` as `core/msg/RobotReport` so the autonomy manager can
track robot state and safety:

```bash
ros2 topic info /robot_report
```

Fields used by this package:

| Field | Used for |
| --- | --- |
| `robot_state`, `robot_state_name` | Mirrors core robot state into autonomy status. |
| `physical_estop`, `comm_estop` | Forces autonomy to `IDLE` while either is active. |
| `comm_fault` | Marks autonomy degraded/error. |
| `error_reason` | Propagates the core error reason into autonomy status. |

For normal robot state tracking, `/robot_report` controls only the `IDLE`/`DRIVE`
autonomy modes. Robot states `READY`, `STAND`, `FLAT_DRIVE`, `ROUGH_DRIVE`, and
`CUSTOM_DRIVE` (`robot_state` 2-6) select autonomy `DRIVE`. Robot states `IDLE`,
`INIT`, `FREEZE`, `SIT`, and `LIE` (`robot_state` 0, 1, 7, 8, 9) select autonomy
`IDLE`. `ADAS`, `FSD`, and `MAPPING` remain service-requested modes.

### Autonomy status

Subscribe to `/autonomy_manager/status` to know what the stack is doing:

```bash
ros2 topic echo /autonomy_manager/status
```

Key fields:

| Field | Meaning |
| --- | --- |
| `mode`, `mode_name` | Effective autonomy mode. |
| `autonomy_enabled` | `true` when mode is not `IDLE` or `ERROR`. |
| `estop_active` | `true` while robot report ESTOP is active. |
| `error_active`, `error_code`, `error_reason` | Error/degraded state. |
| `ai_enabled`, `segmentation_enabled` | Current feature toggles from the mode request. |
| `speed_limit` | Requested speed limit. |
| `pose`, `velocity` | Latest odometry-derived pose and velocity. |
| `active_nodes`, `inactive_nodes`, `degraded_nodes` | Manager view of child-node health. |

## Localization

The autonomy launch uses OpenVINS `ov_msckf` for stereo-inertial VIO instead of
building ORB-SLAM3 in this package. `scripts/build.sh` prepares `src/open_vins` when it
is missing and builds `ov_msckf` together with `autonomy`.

OpenVINS publishes odometry on `/odomimu`. The local
`vio_pose_adapter_node` republishes `/localization/current_pose` as
`geometry_msgs/Pose2D` for the RL local planner and reports the
`openvins_vio_node` heartbeat to the autonomy manager.
