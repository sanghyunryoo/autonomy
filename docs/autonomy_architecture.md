# Autonomy Package Skeleton

This package keeps the existing elevation stack intact and adds an autonomy layer
around it.

## Layers

- Sensors and elevation: RealSense USB mapping, point cloud merge, elevation map,
  local terrain map.
- Perception: ADAS front-camera detector publishes annotated images and
  `DetectedObjectArray` with optional 3D positions.
- Localization: OpenVINS `ov_msckf` runs stereo-inertial VIO and the local
  adapter republishes the current 2D pose for the planner.
- Planning: global planner publishes a global path; ONNX RL local planner consumes
  current pose, target pose, and local height scan, then publishes velocity.
- Supervision: autonomy manager owns mode state, ESTOP, node heartbeat status, and
  degraded/error reporting.

Core autonomy nodes are C++ executables. Python files in this repository are kept
for ROS launch and offline generation/utilities only.

## Modes

- IDLE: no autonomy-critical nodes required.
- DRIVE: elevation stack only.
- ADAS: elevation stack plus front-camera AI detection.
- FSD: elevation stack, AI detection, VIO, global planner, RL local planner.
- MAPPING: elevation stack plus VIO/localization.
- ERROR: internal fault reporting state; it is not requested through
  `/autonomy_manager/set_mode`.
- IDLE/DRIVE effective mode follows `/robot_report`: robot states 2-6 map to
  DRIVE and states 0, 1, 7, 8, 9 map to IDLE. ADAS/FSD/MAPPING are selected by
  service request.
- ESTOP is a safety latch, not an operation mode. When active, the effective mode
  is forced to IDLE to reduce board resource usage. Clearing ESTOP restores the
  previously requested mode.

## Manager Contract

Managed nodes publish heartbeat strings on:

`/autonomy/heartbeat/<node_name>`

The manager publishes:

`/autonomy_manager/state`

The manager consumes robot safety status from:

`/robot_report`

Height-map command gating is published on:

`/command_filter`

Mode changes use:

`/autonomy_manager/set_mode`

ESTOP changes use:

`/autonomy_manager/set_estop`
