# System Architecture

Updated: 2026-07-05

This project has two tracks:

- Gazebo-first development for fast controller and field iteration.
- Physical-car architecture for the final project demonstration.

The Gazebo race launch must stay light and fast. The physical architecture must still be planned clearly so the project does not become only a hard-coded WRO course runner.

## Target Physical Stack

```text
wheel encoders / motor / sensors
        ↓
MCU reads pulses and controls motor/steering
        ↓
CAN bus telemetry and commands
        ↓
main computer running ROS 2
        ↓
/odom /imu /scan /tf
        ↓
SLAM mapping and localization
        ↓
Nav2 planning and obstacle avoidance
        ↓
/cmd_vel
        ↓
MCU motor control
        ↓
Foxglove / RViz debugging
```

## ROS 2 Responsibilities

| Layer | Planned package | Responsibility |
|---|---|---|
| Simulation | `wro2026_sim` | Gazebo world, race launch, line detection, LiDAR obstacle avoidance, simulated odom |
| Physical bringup | `wro2026_bringup` | Future real-car launch, sensor startup, lifecycle wiring |
| CAN bridge | `wro2026_can_bridge` | Future ROS 2 node translating CAN frames to encoder, motor, battery, and status topics |
| Robot model | `wro2026_description` | Future URDF, frames, sensor poses, wheelbase, base footprint |
| Navigation | `wro2026_navigation` | Future maps, SLAM Toolbox config, Nav2 costmaps, planner/controller params |

Only `wro2026_sim` is active in the current Gazebo workflow. Mapping is launched through `wro2026_sim/launch/mapping.launch.py` when needed. The other packages are planned integration boundaries.

## Current Gazebo Data Flow

```text
Gazebo world
  ├─ /camera/image_raw ──> line_detector.py ──> /line_detection
  ├─ /scan ───────────────────────────────┐
  ├─ dynamic pose ─> gazebo_ground_truth_odom.py ─> /odom and /tf
  └─ /cmd_vel <── line_follower.py <──────┘
```

`line_follower.py` currently combines:

- LiDAR wall centering
- orange/blue line-triggered cornering
- odom-yaw 90 degree turn completion
- 160 cm field speed zones
- upper-section LiDAR gap avoidance

## Physical Data Flow Plan

```text
encoder ticks + IMU + sensors
        ↓
MCU firmware
        ↓
CAN frames
        ↓
wro2026_can_bridge
        ↓
/wheel_ticks /motor_state /battery_state /cmd_motor
        ↓
robot_localization + TF
        ↓
/odom /imu /tf
        ↓
SLAM / Nav2 / race controller
        ↓
/cmd_vel
```

The physical car should not depend on Gazebo ground truth. The simulation keeps ground truth odom only to accelerate controller development.

## Debugging Interfaces

Use RViz/Foxglove to inspect:

- `/scan`
- `/odom`
- `/tf`
- `/cmd_vel`
- `/line_detection`
- future `/imu`
- future `/wheel_ticks`
- future `/can_status`

The final demonstration should show both behavior and data: the car driving, plus live topics proving the stack is working.
