# System Architecture

Updated: 2026-10-13 (real-car packages and data flow added; the 2026-07-05 plan below is kept)

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
| Chassis (transitional car) | `wro2026_base_waverover` | **Exists (2026-10).** WAVE ROVER driver: `/cmd_vel` -> ESP32 JSON over HTTP or UART, `/battery`, `/imu/data`, `/diagnostics`, low-quality dead-reckoning `/odom` + TF `odom -> base_link`. Replaced when the competition chassis arrives |
| Real-car upper layer | `wro2026_real` | **Exists (2026-10).** Bringup (driver + robot_state_publisher), minimal URDF, parameterised line detector. Talks to the chassis only through `/cmd_vel`, `/odom`, TF and sensor topics |
| Physical bringup | `wro2026_bringup` | Future real-car launch, sensor startup, lifecycle wiring (for now covered by `wro2026_real`) |
| CAN bridge | `wro2026_can_bridge` | Future ROS 2 node translating CAN frames to encoder, motor, battery, and status topics |
| Robot model | `wro2026_description` | Future URDF, frames, sensor poses, wheelbase, base footprint |
| Navigation | `wro2026_navigation` | Future maps, SLAM Toolbox config, Nav2 costmaps, planner/controller params |

`wro2026_sim` is the Gazebo package. On the real car (Jetson), only `wro2026_base_waverover` and `wro2026_real` are built; `wro2026_sim` must not be deployed there. Mapping is launched through `wro2026_sim/launch/mapping.launch.py` when needed. The other packages are planned integration boundaries.

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

## Current Real-Car Data Flow (2026-10, Waveshare WAVE ROVER)

The transitional car has an ESP32 board with a JSON protocol and **no wheel encoders**, so the plan above (MCU + CAN + encoders) is not available yet.

```text
teleop / future controller ──/cmd_vel──┐            (nothing publishes /cmd_vel by default)
                                       ▼
                    wro2026_base_waverover (Jetson)
                      core: kinematics, max_speed cap, cmd_timeout,
                            feedback watchdog, battery limits, odom
                      transport: HTTP (Wi-Fi) or UART /dev/ttyTHS1
                                       │  {"T":1,"L":..,"R":..}  {"T":130}  {"T":126}
                                       ▼
                    ESP32 on the WAVE ROVER ── 4 motors (PWM, no encoders), IMU
                                       │  {"T":1001,...,"v":..}  {"T":1002,r/p/y,a*,g*}
                                       ▼
   /battery  /imu/data  /diagnostics  /wave_rover/feedback  /wave_rover/sent_cmd
   /odom + TF odom -> base_link   (v = sent wheel command x pwm_to_mps [UNKNOWN, 0];
                                   wz = IMU gz; held still when the command is zero)
                                       │
   wro2026_real: robot_state_publisher (urdf/wave_rover.urdf)
                 TF base_link -> imu_link, camera_link (placeholder poses)

   camera (model UNKNOWN, no driver yet) ──/camera/image_raw──> wro2026_real line_detector
                                                               ──/line_detection (9 floats)
```

The `/line_detection` layout is the simulation's Float32MultiArray and is fragile; it should become a custom message. The dead-reckoning odometry is low quality (large covariance) and must not be the main odometry source for Nav2.

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
