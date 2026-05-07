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
- `srv/CommandFilter`: checks the latest height map for blocking obstacles in
  the requested forward/right movement corridor.
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

For simulation, publish depth images and camera info on the topics configured in
`config/realsense_usb_mapping.yaml` under `camera_bindings.simulation`. This
skips the hardware mapper and enables `use_sim_time` on the merge and
elevation nodes. Only camera bindings with `enabled: true` are subscribed and
merged:

```bash
ros2 launch height_map_ros2 multi_realsense_elevation.launch.py simulation:=true
```

For hardware, keep USB port bindings and camera merge settings in
`config/realsense_usb_mapping.yaml` under `camera_bindings.real`.
First identify the board port ids:

```bash
sudo apt install python3-opencv python3-yaml
python3 -m pip install pyrealsense2
ros2 run height_map_ros2 show_realsense_usb_ports.py
```

Each connected RealSense color image is displayed with its `usb_port_id`, serial,
and model overlaid. Plug a camera into each board port, note the reported
`usb_port_id`, then edit `config/realsense_usb_mapping.yaml`. After that, any
camera plugged into that physical port is mapped to the configured role while
`enabled` still controls whether the role is used.

The hardware mapper resolves configured USB ports to the currently connected
device serials, publishes per-camera depth images plus `CameraInfo` at the
configured depth FPS, and publishes the resolved bindings as JSON on
`~/camera_bindings`.

```bash
ros2 launch height_map_ros2 multi_realsense_elevation.launch.py simulation:=false
```

Select the camera/resource profile with `operation_mode:=drive`, `adas`, or
`fsd`. `drive` starts only the configured drive camera roles. `adas` and `fsd`
also require an enabled `role: adas` camera; launch or the hardware mapper will
fail loudly if that role is missing. In `adas` and `fsd`, the elevation node also
publishes `~/local_terrain_map`, `~/local_terrain_image`, and
`~/local_terrain_points` using the wider `local_terrain_map.grid` bounds.

The elevation config stays hardware-independent. Edit
`config/realsense_usb_mapping.yaml` for camera USB port roles, enabled flags, topic
names, TF frame topology, and merge filter settings. Edit
`config/elevation_mapping.yaml` for merged-cloud input/output, grid bounds, and
elevation algorithm tuning.

## DDS output

The DDS-facing IDL is installed as `share/height_map_ros2/idl/HeightMap.idl`:

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

Call `~/command_filter` with `move_forward` and `move_right`. A response field is
`false` when the latest height map contains an obstacle at least `0.25 m` above
the configured robot floor in that movement corridor; otherwise it is `true`.
