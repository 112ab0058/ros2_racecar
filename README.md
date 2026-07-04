# WRO 2026 Future Engineers - Self-Driving Car

**Team:** NTUT
**Country:** Taiwan
**Current focus:** 160 x 160 cm project field, Gazebo first, physical-car architecture planned

## Project Overview

This repository contains the ROS 2 and Gazebo code for a WRO Future Engineers inspired self-driving car project.

The current project field is a fixed 1600 x 1600 mm test area. It keeps orange/blue line-triggered turns from the older WRO-style work, adds a south high-speed straight, adds a north continuous obstacle zone, and reserves an east parking/docking zone.

The first implementation goal is not to jump straight into AI or Nav2. The current main flow is:

```text
Gazebo camera -> line_detector.py -> /line_detection
Gazebo LiDAR + /odom + /line_detection -> line_follower.py -> /cmd_vel
```

The longer physical-car route is still part of the project:

```text
encoder / motor / sensors
        -> MCU
        -> CAN bus
        -> ROS 2 main computer
        -> /odom /imu /scan /tf
        -> SLAM and Nav2
        -> /cmd_vel
        -> MCU motor control
        -> Foxglove / RViz debugging
```

See `docs/SYSTEM_ARCHITECTURE.md` for the full architecture plan.

## Current Stack

| Area | Current choice |
|---|---|
| Simulation OS | WSL2 Ubuntu 24.04 |
| ROS 2 simulation | Jazzy |
| Gazebo | Harmonic / gz-sim v8 |
| Robot model | TurtleBot3 Burger camera model |
| Race control | HSV line detection + LiDAR wall centering + LiDAR gap avoidance |
| Mapping workflow | Separate `mapping.launch.py` with SLAM Toolbox |
| Physical compute | Jetson Orin Nano planned |
| Physical control | MCU + encoder/IMU/CAN planned |

## Active Package

Only one ROS package is active in the mainline now:

```text
src/wro2026_sim
```

Important files:

```text
src/wro2026_sim/worlds/wro2026_field.sdf
src/wro2026_sim/launch/race.launch.py
src/wro2026_sim/launch/mapping.launch.py
src/wro2026_sim/launch/physical_bringup.launch.py
src/wro2026_sim/scripts/gazebo_ground_truth_odom.py
src/wro2026_sim/scripts/line_detector.py
src/wro2026_sim/scripts/line_follower.py
src/wro2026_sim/config/slam_params.yaml
src/wro2026_sim/models/turtlebot3_burger_cam
```

Planned future package boundaries:

```text
wro2026_bringup
wro2026_can_bridge
wro2026_description
wro2026_navigation
```

These are documented but not created as active code yet, so the simulation stays small and easy to run.

## Field

The official Gazebo world is:

```text
src/wro2026_sim/worlds/wro2026_field.sdf
```

Current dimensions:

| Element | Specification |
|---|---|
| Field size | 1600 x 1600 mm |
| Central island | 400 x 400 mm |
| Nominal lane width | about 600 mm |
| Corner radius target | R80 mm approximation |
| High-speed straight | about 1100 mm on south side |
| Continuous obstacle zone | north side, movable obstacles |
| Parking / docking zone | east side |
| Orange / blue line width | 20 mm |
| Simulation wall height | 300 mm for TurtleBot3 LiDAR |

See `docs/FIELD_PLAN_160CM.md` for physical material and layout notes.

## Runtime Modes

### Race Mode

Race mode is the normal controller test.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch wro2026_sim race.launch.py
```

It starts:

- Gazebo server-only: `gz sim -r -s`
- bridge topics: `/cmd_vel`, `/scan`, `/clock`, `/camera/image_raw`, `/camera/camera_info`
- `gazebo_ground_truth_odom.py`
- `line_detector.py`
- `line_follower.py`

Race mode does not start SLAM or Nav2, because WSL performance matters during controller tuning.

### Mapping Mode

Mapping mode is only for map generation and geometry checks.

```bash
ros2 launch wro2026_sim mapping.launch.py
```

Do not run mapping mode together with race mode.

### Future Physical Bringup

The placeholder launch entry is:

```bash
ros2 launch wro2026_sim physical_bringup.launch.py
```

It currently records the intended integration point for encoder, IMU, CAN bridge, MCU command output, robot description, and future navigation.

## Controller Status

Current controller:

```text
src/wro2026_sim/scripts/line_follower.py
```

Main states:

```text
STRAIGHT -> APPROACH -> TURN -> ALIGN -> POST_TURN -> STRAIGHT
```

Obstacle states:

```text
AVOID_OBSTACLE -> RETURN_TO_LINE -> STRAIGHT
```

Current behavior:

- Orange/blue lines trigger turns.
- Ground-truth `/odom` is used in simulation for stable yaw-based 90 degree turns.
- LiDAR wall centering keeps the vehicle inside the lane.
- South straight uses a faster speed zone.
- North obstacle zone uses a slower LiDAR gap-selection avoidance mode.
- Emergency recovery still stops/reverses only when the obstacle is too close.

AI is intentionally not in the first control loop. YOLO can be added later for red/green pillars or parking/station markers, but it should not directly control the car in the first stable version.

## Gazebo Performance Defaults

Race mode is optimized for WSL:

- server-only Gazebo by default
- `max_step_size = 0.005`
- simple box/cylinder primitives
- no SLAM/Nav2 in race launch
- camera debug image disabled by default
- camera model fixed at 320 x 240, 10 Hz
- LiDAR model fixed at 360 samples, 8 Hz
- only required bridge topics enabled

See `docs/GAZEBO_PERFORMANCE.md` for the performance checklist.

## Repository Structure

```text
ros2_racecar/
├── README.md
├── docs/
│   ├── CURRENT_STATE.md
│   ├── FIELD_PLAN_160CM.md
│   ├── GAZEBO_PERFORMANCE.md
│   ├── ROS2_WSL_NETWORK.md
│   ├── SYSTEM_ARCHITECTURE.md
│   └── WORKSPACE_MAP.md
├── ros2_wsl_network.bash
└── src/
    └── wro2026_sim/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/slam_params.yaml
        ├── launch/
        │   ├── race.launch.py
        │   ├── mapping.launch.py
        │   └── physical_bringup.launch.py
        ├── maps/.gitkeep
        ├── scripts/
        │   ├── gazebo_ground_truth_odom.py
        │   ├── line_detector.py
        │   └── line_follower.py
        └── worlds/wro2026_field.sdf
```

The old tutorial packages, loose demo scripts, old 3.2 m maps, old mapper package, and old world variants are intentionally removed from the mainline.

## Next Work

1. Run `race.launch.py` and verify the 160 cm world starts smoothly.
2. Tune line detection positions and turn parameters until the car can complete 3 laps without obstacles.
3. Test one obstacle, then three obstacles, in the north continuous obstacle zone.
4. Tune speed zones so the south straight is visibly faster and corners remain controlled.
5. Add parking/docking detection and action.
6. Create the future physical packages when encoder, IMU, CAN, and MCU interfaces are ready.
