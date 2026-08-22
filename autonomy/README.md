# autonomy

Jetson autonomy runtime organized around five managers in one 50 Hz cycle:

- `ParameterManager`: loads and validates configuration/URDF paths before all runtime managers are created, then supplies one `RuntimeConfig` to them.
- `SensorManager`: owns direct librealsense2/Livox-SDK2 acquisition, parses URDF static TF, and merges points in `target_frame`.
- `StateManager`: selects IDLE/ADAS from `core/RobotReport`; only ROS services can select FSD, MAPPING, or TRACKING.
- `AlgorithmManager`: owns ROS-free elevation, Super-LIO, occupancy mapping, Hybrid A*, MPPI, and RGB-D detection. Each algorithm uses its configured `hz` as both update rate and execution deadline; a missed deadline is reported as a Middleware warning, while a calculation failure returns an error code and stops the loop.
- `MiddlewareManager`: owns ROS 2, Cyclone DDS, and `core-msg` endpoints;
  it exposes non-blocking `poll()` and `publish()` operations only.

`RuntimeRunner` is the composition root, not a sixth manager. It loads
`ParameterManager` first, creates SensorManager before MiddlewareManager because
the middleware bridge attaches to the sensor inventory, then creates StateManager
and AlgorithmManager with the validated `RuntimeConfig`. The explicit 50 Hz loop
is always `Middleware input → State → (non-IDLE) Sensor snapshot/merge → due
algorithms → Middleware output`; `MiddlewareManager::poll()` only drains ready
ROS callbacks and never controls the loop.

Robot reports and `~/set_mode` service requests are both queued by Middleware
and consumed by `RuntimeRunner::processMiddlewareInputs()` before StateManager
ticks. The service immediately acknowledges queueing; the requested mode is
validated and applied in the following 50 Hz control cycle.

`main.cpp` is intentionally a thin entry point: it creates and runs only
`RuntimeRunner`. `ParameterManager` takes the configuration directory from
`AUTONOMY_CONFIG_DIR`; `scripts/launch.sh` sets it from `config_dir:=...`.

`reference/` and `autonomy-main/` are preserved only as read-only references. Super-LIO is embedded in `src/algorithm/slam`; Nav2 is not a dependency. This package is GPL-3.0-only because of the Super-LIO-derived core.

All algorithm implementation and public headers now follow the same four-domain layout:

```text
include/autonomy/algorithm/{slam,planner,detection,elevation}/
src/algorithm/{slam,planner,detection,elevation}/
```

`AlgorithmManager` only schedules these implementations and merges their results; it does not contain an algorithm implementation itself. The only top-level code domains are `algorithm`, `middleware`, `sensor`, `core`, and `parameter`. `core` contains the FSM-oriented StateManager and RuntimeRunner.

The runtime configuration is separated by manager responsibility: [core.yaml](include/autonomy/parameter/config/core.yaml), [sensor.yaml](include/autonomy/parameter/config/sensor.yaml), [algorithm.yaml](include/autonomy/parameter/config/algorithm.yaml), and [middleware.yaml](include/autonomy/parameter/config/middleware.yaml). `sensor.yaml` configures direct SDK inventory: RealSense model/serial-or-USB-port/stream geometry and MID-360/MID-360S model/IP/SDK JSON. No ROS topic is used to acquire a sensor. Jetson defaults are lidar `192.168.1.50`, SBC interface `192.168.20.2`, and SBC peer `192.168.20.1`.

`algorithm.yaml` defines `elevation.hz`, `slam.hz`, `planner.hz`, and
`detection.hz` (default: 50, 10, 10, and 30 Hz). A due result is published by Middleware only after the matching
algorithm has completed; taking longer than that algorithm's period emits a
terminal warning with actual and target processing time in milliseconds, plus a
`warning` node status.

`middleware.yaml` separates `ros2` and `dds`. ROS 2 debug output is disabled by
default. When `middleware.ros2.debug.enabled: true`, it sends only bounded,
best-effort `/autonomy/debug/...` streams at the configured 2 Hz: elevation as
PointCloud2, one transient sensor-TF message, SLAM odometry and pose path, and
decimated Livox PointCloud2, plus raw `ai_realsense` RGB and its inference bbox
overlay. Point clouds are capped at 2,000 points and the SLAM path at 200 poses
by default; AI images are capped at 320×240. `ai_realsense` remains disabled
until its actual librealsense serial or USB physical port is set in
`sensor.yaml`. Invalid sensor input, SLAM/mapping failure,
unreachable goal, or MPPI collision failure latches an autonomy error immediately.

```bash
./scripts/build.sh --workspace /path/to/ros2_ws
./scripts/launch.sh
ros2 service call /autonomy_middleware_manager/set_mode autonomy/srv/SetAutonomyMode \
  "{operation_mode: FSD, speed_limit: 0.0, enable_ai: false, segmentation: false}"
```
