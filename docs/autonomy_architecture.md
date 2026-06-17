# Autonomy Package Skeleton

This package keeps the existing elevation stack intact and adds an autonomy layer
around it.

## Layers

- Sensors and elevation: RealSense USB mapping, point cloud merge, elevation map,
  local terrain map.
- Perception: ADAS front-camera detector publishes annotated images and
  `DetectedObjectArray` with optional 3D positions.
- Localization: Point-LIO is the only SLAM backend; the local adapter republishes
  Point-LIO odometry as the current 2D pose for planner consumers.
- Planning: Nav2 handles navigation behaviors from Point-LIO localization and the
  saved/global costmap; the command bridge converts Nav2 velocity into user commands.
- Supervision: autonomy manager owns mode state, ESTOP, node heartbeat status, and
  degraded/error reporting.

Core autonomy nodes are C++ executables. Python files in this repository are kept
for ROS launch and offline generation/utilities only.

## Modes

- IDLE: no autonomy-critical nodes required.
- DRIVE: elevation stack only.
- ADAS: elevation stack plus front-camera AI detection.
- FSD: elevation stack, AI detection, RGB-D localization, global planner, RL local planner.
- MAPPING: elevation stack plus RGB-D localization.
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

ESTOP comes from `/robot_report`:

`physical_estop || comm_estop`
