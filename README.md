# height_map_ros2

ROS2 integration layer for merging multiple RealSense point clouds into `base_link`
and producing a robot-centric elevation image.

## Structure

- `realsense_serial_mapper.py`: hardware-only node that opens RealSense devices
  by serial number and publishes per-camera depth images plus `CameraInfo`.
- `PointCloudMergerNode`: subscribes to each camera depth image and intrinsics,
  publishes static TF from the URDF, back-projects depth maps into 3D, transforms
  points into `target_frame`, and publishes `~/merged_points`.
- `ElevationMappingNode`: subscribes to the merged point cloud and publishes
  `~/elevation_image` (`sensor_msgs/Image`, `32FC1`).
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
`config/realsense_serial_mapping.yaml` under `camera_bindings.simulation`. This
skips the hardware serial mapper and enables `use_sim_time` on the merge and
elevation nodes. Only camera bindings with `enabled: true` are subscribed and
merged:

```bash
ros2 launch height_map_ros2 multi_realsense_elevation.launch.py simulation:=true
```

For hardware, keep serial numbers and camera merge settings in
`config/realsense_serial_mapping.yaml` under `camera_bindings.real`.
First identify physical cameras:

```bash
sudo apt install python3-opencv python3-yaml
python3 -m pip install pyrealsense2
ros2 run height_map_ros2 show_realsense_serials.py
```

Each connected RealSense color image is displayed with its serial number overlaid.
Use the image direction to decide which physical camera is `front`, `rear`, etc.,
then edit `config/realsense_serial_mapping.yaml`.

The serial mapper opens configured devices by serial number, publishes per-camera
depth images plus `CameraInfo` at the configured depth FPS, and publishes the
resolved bindings as JSON on `~/camera_bindings`.

```bash
ros2 launch height_map_ros2 multi_realsense_elevation.launch.py simulation:=false
```

The elevation config stays hardware-independent. Edit
`config/realsense_serial_mapping.yaml` for camera identity, enabled flags, topic
names, TF frame topology, and merge filter settings. Edit
`config/elevation_mapping.yaml` for merged-cloud input/output, grid bounds, and
elevation algorithm tuning.
