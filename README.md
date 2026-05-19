# height_map_ros2

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
- `msg/CommandFilter`: publishes movement-axis allow/deny decisions from the
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

From a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
ln -s /home/sunrise/SH/height_cli_lib_new/ros2/height_map_ros2 ~/ros2_ws/src/height_map_ros2
cd ~/ros2_ws
colcon build --packages-select height_map_ros2
source install/setup.bash
```

## Run

The autonomy stack is launched from a single launch file and a single parameter
file, `resources/config/autonomy.yaml`. On startup the stack stays in `IDLE`; change modes
at runtime with `/autonomy_manager/set_mode`.

Simulation:

```bash
ros2 launch height_map_ros2 autonomy.launch.py simulation:=true
```

Hardware:

```bash
ros2 launch height_map_ros2 autonomy.launch.py simulation:=false
```

Keep USB port bindings, camera topic bindings, merge settings, elevation tuning,
SLAM, AI, planner, and manager parameters in `resources/config/autonomy.yaml`. For
hardware, first identify the board port ids:

```bash
sudo apt install python3-opencv python3-yaml
python3 -m pip install pyrealsense2
ros2 run height_map_ros2 show_realsense_usb_ports.py
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
ros2 service call /autonomy_manager/set_mode height_map_ros2/srv/SetAutonomyMode \
"{operation_mode: 'ADAS', speed_limit: 0.8, enable_ai: true, segmentation: false}"
```

Trigger ESTOP:

```bash
ros2 service call /autonomy_manager/set_estop height_map_ros2/srv/SetEstop \
"{active: true, reason: 'operator'}"
```

Clear ESTOP and restore the previously requested mode:

```bash
ros2 service call /autonomy_manager/set_estop height_map_ros2/srv/SetEstop \
"{active: false, reason: 'operator'}"
```

Monitor the manager status:

```bash
ros2 topic echo /autonomy_manager/status
```

## Localization

The autonomy launch uses OpenVINS `ov_msckf` for stereo-inertial VIO instead of
building ORB-SLAM3 in this package. `scripts/build.sh` prepares `src/open_vins` when it
is missing and builds `ov_msckf` together with `height_map_ros2`.

OpenVINS publishes odometry on `/odomimu`. The local
`vio_pose_adapter_node` republishes `/localization/current_pose` as
`geometry_msgs/Pose2D` for the RL local planner and reports the
`openvins_vio_node` heartbeat to the autonomy manager.

## DDS output

The DDS-facing IDL is installed as `share/height_map_ros2/resources/idl/HeightMap.idl`:

```idl
module core_dds {
  struct HeightMap {
    sequence<float> data;
  };
};
```

By default the node writes:

- domain id: `0`
- topic: `height_map`
- type: `core_dds::HeightMap`
- sample: `data` with the current grid length
- QoS: keep last depth `1`, best effort

The sample length follows the configured grid size, so it is not hard-coded to
`144`.

## Command filter

The elevation node publishes `/command_filter` as `height_map_ros2/msg/CommandFilter`:

- `allow_linear_vel_x`
- `allow_linear_vel_y`

A field is `false` when the latest height map contains an obstacle at least
`0.25 m` above the configured robot floor in that movement corridor; otherwise
it is `true`. If no height map is available, both fields default to `true`.

## Robot Report

The autonomy manager subscribes to `/robot_report` as
`height_map_ros2/msg/RobotReport`. `physical_estop` or `comm_estop` forces the
effective autonomy mode to `IDLE`; clearing the report ESTOP restores the
previously requested mode unless operator ESTOP is still active. `comm_fault`
marks autonomy status degraded/error and is reflected in `/autonomy_manager/status`.

For normal robot state tracking, `/robot_report` controls only the `IDLE`/`DRIVE`
autonomy modes. Robot states `READY`, `STAND`, `FLAT_DRIVE`, `ROUGH_DRIVE`, and
`CUSTOM_DRIVE` (`robot_state` 2-6) select autonomy `DRIVE`. Robot states `IDLE`,
`INIT`, `FREEZE`, `SIT`, and `LIE` (`robot_state` 0, 1, 7, 8, 9) select autonomy
`IDLE`. `ADAS`, `FSD`, and `MAPPING` remain service-requested modes.
