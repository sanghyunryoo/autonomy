# Autonomy Package Skeleton

This package keeps the existing elevation stack intact and adds an autonomy layer
around it.

## Layers

- Sensors and elevation: RealSense USB mapping, point cloud merge, elevation map,
  local terrain map.
- Perception: ADAS front-camera detector publishes annotated images and
  `DetectedObjectArray` with optional 3D positions.
- Localization and mapping: ORB-SLAM3 adapter publishes the current pose.
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
- FSD: elevation stack, AI detection, SLAM, global planner, RL local planner.
- MAPPING: elevation stack plus SLAM.
- ERROR: no autonomous actuation.
- ESTOP: emergency stop state.

## Manager Contract

Managed nodes publish heartbeat strings on:

`/autonomy/heartbeat/<node_name>`

The manager publishes:

`/autonomy_manager/state`

Mode changes use:

`/autonomy_manager/set_mode`

ESTOP changes use:

`/autonomy_manager/set_estop`
