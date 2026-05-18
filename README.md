# WRO 2026 Future Engineers — Self-Driving Car

**Team:** NTUT
**Country:** Taiwan
**Season:** WRO 2026 Future Engineers — Self-Driving Cars

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Hardware Platform](#2-hardware-platform)
3. [Software Architecture](#3-software-architecture)
4. [Game Field and Simulation](#4-game-field-and-simulation)
5. [Mobility and Sensor Design](#5-mobility-and-sensor-design)
6. [Obstacle Management Strategy](#6-obstacle-management-strategy)
7. [Repository Structure](#7-repository-structure)
8. [How to Build and Run](#8-how-to-build-and-run)
9. [Engineering Decisions and Tradeoffs](#9-engineering-decisions-and-tradeoffs)
10. [Team Photos and Vehicle Photos](#10-team-photos-and-vehicle-photos)
11. [Videos](#11-videos)

---

## 1. Project Overview

This repository contains the complete source code, simulation worlds, configuration files, and documentation for our WRO 2026 Future Engineers Self-Driving Car entry.

The challenge requires a fully autonomous robotic vehicle to complete three laps on a randomly configured racetrack, detect and respond to colored traffic sign pillars, and perform parallel parking after finishing the laps. The track corridor width varies between 600 mm and 1000 mm and is randomized before each round.

Our solution is built on the following technical stack:

- **Compute platform:** NVIDIA Jetson Orin Nano 8GB running Ubuntu 22.04 with JetPack 6.2
- **Robot Operating System:** ROS 2 Humble on the physical vehicle; ROS 2 Jazzy in simulation
- **Simulation environment:** Gazebo Harmonic (gz-sim v8) running in WSL2 Ubuntu 24.04
- **Mapping:** SLAM Toolbox (online async mode) with a custom ground-truth odometry node
- **Perception:** OpenCV HSV-based color detection for orange and blue turn lines; YOLOv8 for red and green pillar detection (in development)
- **Planning and control:** Rule-based finite state machine combining line detection, pillar avoidance, and parking logic

The development workflow separates simulation on a Windows laptop (WSL2) from deployment on the physical Orin Nano device, using CycloneDDS peer-to-peer mode for cross-machine ROS 2 communication.

---

## 2. Hardware Platform

### Main Compute Board

| Item | Detail |
|------|--------|
| Board | NVIDIA Jetson Orin Nano 8GB |
| OS | Ubuntu 22.04 + JetPack 6.2 |
| Storage | 28 GB eMMC |
| Remote access | SSH + VNC over Tailscale |

The Orin Nano was selected because it provides sufficient GPU compute for real-time camera inference (YOLOv8) while staying within the vehicle size and weight constraints which are no more than 300 x 200 x 300 mm and no more than 1.5 kg.

### Sensors

| Sensor | Purpose |
|--------|---------|
| Wide-angle camera | Orange/blue line detection, red/green pillar detection |
| LiDAR (simulation only) | SLAM mapping in simulation; not usable on physical vehicle because the real walls are only 100 mm tall |

A key hardware constraint is that the physical game field walls are only 100 mm tall, which is below the scan plane of most LiDAR sensors mounted at the standard height on TurtleBot3 Burger at approximately 180 mm. This means that SLAM-based navigation is only practical in simulation where we set wall height to 300 mm. On the physical vehicle, all navigation relies on camera-based perception.

### Vehicle Chassis

The physical vehicle chassis dimensions must not exceed 300 x 200 mm footprint and 300 mm height. The drive system uses a differential drive configuration for simulation using TurtleBot3 Burger model. The final physical vehicle uses a dedicated steering servo and DC motor with encoder.

---

## 3. Software Architecture

The software is divided into the following modules:

- **wro2026_sim** — Gazebo world files, launch, SLAM config, maps
- **wro2026_mapper** — Autonomous mapping node with lawn-mower pattern
- **line_detector** — Orange/blue line detection via OpenCV HSV (Step 5, in development)
- **pillar_detector** — Red/green pillar detection via YOLOv8 (Step 6, planned)
- **wro2026_controller** — Finite state machine for line following and pillar avoidance (Step 7, planned)
- **parking_controller** — Parallel parking detection and execution (Step 8, planned)

### ROS 2 Topic Architecture

| Topic | Type | Producer | Consumer |
|-------|------|----------|----------|
| /camera/image_raw | sensor_msgs/Image | Gazebo bridge | line_detector, pillar_detector |
| /scan | sensor_msgs/LaserScan | Gazebo bridge | SLAM Toolbox |
| /odom | nav_msgs/Odometry | gazebo_ground_truth_odom | SLAM Toolbox, controller |
| /cmd_vel | geometry_msgs/Twist | controller | Gazebo bridge to robot |
| /map | nav_msgs/OccupancyGrid | SLAM Toolbox | stored, used for planning |

### Key Design Decision: Ground Truth Odometry

Standard Gazebo odometry drifts because it is computed by integrating wheel encoder data. We implemented a custom node called gazebo_ground_truth_odom.py that reads the model true position directly from the Gazebo pose topic at /world/wro2026/dynamic_pose/info and publishes it as /odom. This eliminates drift entirely in simulation and produces a clean 66 x 66 occupancy grid map.

---

## 4. Game Field and Simulation

### Field Specification

The WRO 2026 game field is a 3200 x 3200 mm square racetrack with a central square obstacle. Key dimensions are listed below.

| Element | Specification |
|---------|--------------|
| Field size | 3200 x 3200 mm |
| Corridor width wide | 1000 mm |
| Corridor width narrow | 600 mm |
| Outer wall height real | 100 mm |
| Outer wall height simulation | 300 mm required for LiDAR |
| Orange line width | 20 mm |
| Blue line width | 20 mm |
| Traffic sign pillar size | 50 x 50 x 100 mm |
| Parking lot limiter | 200 x 20 x 100 mm magenta |

### Orange and Blue Turn Lines

Each corner of the track has two colored lines indicating turn direction. In clockwise driving direction, the sequence NW to NE to SE to SW produces Orange, Blue, Orange, Blue, Orange, Blue, Orange, Blue. The lines are positioned so that each orange line touches one wall of the corner and each blue line touches the other wall.

Line colors match the official specification. Orange is RGB 255 102 0 which corresponds to CMYK 0 60 100 0. Blue is RGB 0 51 255 which corresponds to CMYK 100 80 0 0.

### Simulation World Files

| World file | Description |
|------------|-------------|
| wro2026_field.sdf | Original reference field used for SLAM mapping |
| wro2026_open_wide.sdf | Open Challenge, 1000 mm corridor, no pillars |
| wro2026_open_narrow.sdf | Open Challenge, 600 mm corridor, no pillars |
| wro2026_obstacle_wide.sdf | Obstacle Challenge, 1000 mm corridor, red and green pillars, parking lot |

---

## 5. Mobility and Sensor Design

### Drive System

The simulation uses TurtleBot3 Burger with differential drive. The physical vehicle will use a steering servo on the front axle and a single DC motor with gearbox driving the rear axle.

### Camera Placement

The camera is mounted on the TurtleBot3 Burger chassis at the standard forward-facing position. For line detection, the camera needs to see the floor approximately 200 to 400 mm in front of the vehicle. For pillar detection, it needs a field of view wide enough to see pillars at the sides of the corridor.

### LiDAR vs Camera Navigation

| Method | Simulation | Physical vehicle |
|--------|-----------|-----------------|
| LiDAR SLAM | Used for mapping | Walls too short at 100 mm |
| Camera line detection | In development | Primary navigation |
| Camera pillar detection | Planned YOLOv8 | Required for obstacle challenge |

---

## 6. Obstacle Management Strategy

### Open Challenge

The vehicle detects orange and blue lines on the floor using OpenCV HSV color filtering. The detection logic works as follows. First subscribe to /camera/image_raw. Then convert image from BGR to HSV color space. Apply HSV mask for orange color range and blue color range separately. Compute contour area for each color. If orange line area exceeds threshold then publish turn signal. If blue line area exceeds threshold then determine clockwise or counterclockwise direction.

### Obstacle Challenge

Red and green pillars are detected using a YOLOv8 model fine-tuned on synthetic training images generated from Gazebo simulation renders. A red pillar means keep to the right side of the lane. A green pillar means keep to the left side of the lane.

### Parking

After completing three laps, the vehicle detects the magenta parking lot limiters using HSV color detection and performs parallel parking by executing a fixed sequence of movements. Touching the parking limiters ends the round with zero score, so the parking approach uses conservative speed and distance margins.

---

## 7. Repository Structure
ros2_racecar/
├── README.md
├── .gitignore
├── ros2_wsl_network.bash
├── docs/
│   ├── CURRENT_STATE.md
│   └── ROS2_WSL_NETWORK.md
├── tools/
│   └── fix_lines.py
└── src/
├── wro2026_sim/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/slam_params.yaml
│   ├── launch/wro2026_sim.launch.py
│   ├── maps/
│   ├── scripts/
│   │   ├── gazebo_ground_truth_odom.py
│   │   └── wro2026_mapper.py
│   └── worlds/
│       ├── wro2026_field.sdf
│       ├── wro2026_open_wide.sdf
│       ├── wro2026_open_narrow.sdf
│       └── wro2026_obstacle_wide.sdf
└── wro2026_mapper/
├── package.xml
├── setup.py
├── setup.cfg
└── wro2026_mapper/wro2026_mapper.py
---

## 8. How to Build and Run

### Prerequisites for Simulation on WSL2 Ubuntu 24.04

- ROS 2 Jazzy
- Gazebo Harmonic gz-sim v8
- ros-jazzy-ros-gz-bridge
- ros-jazzy-slam-toolbox
- ros-jazzy-turtlebot3-gazebo
- ros-jazzy-robot-state-publisher

### Network Setup for WSL2

Every time WSL2 is restarted, run the following commands.

```bash
source ~/ros2_ws/ros2_wsl_network.bash
ros2_wsl_fix_loopback
ros2 daemon start
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run Simulation

```bash
ros2 launch ~/ros2_ws/src/wro2026_sim/launch/wro2026_sim.launch.py
```

Gazebo starts automatically with auto-play enabled. Wait approximately 15 seconds for all nodes to initialize. SLAM Toolbox starts last and begins building the map automatically.

### Cross-machine ROS 2 between WSL2 and Orin Nano

```bash
ros2_wsl_peers <Orin-Nano-IP>
```

Both machines must use the same ROS_DOMAIN_ID.

---

## 9. Engineering Decisions and Tradeoffs

### Why CycloneDDS instead of FastDDS

WSL2 network isolation makes DDS multicast unreliable. FastDDS with ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET caused ros2 node list and daemon commands to hang indefinitely after WSL restarts. We switched to CycloneDDS bound to the loopback interface for local simulation work, with named shell functions to switch between profiles when cross-machine communication is needed.

### Why ground truth odometry instead of wheel encoder odometry

TurtleBot3 Burger differential drive odometry accumulates drift over time. After two full laps of the 3.2 x 3.2 m field, position error was large enough to corrupt the SLAM map. The ground truth odometry node reads the model true position from Gazebo dynamic_pose topic, eliminating drift completely in simulation and producing a clean 66 x 66 occupancy grid.

### Why camera navigation instead of LiDAR on the physical vehicle

The WRO 2026 game field uses 100 mm tall walls. A LiDAR mounted at the standard height on TurtleBot3 Burger at approximately 180 mm scans above the walls and cannot detect them. SLAM-based wall following is therefore not viable on the physical vehicle. All navigation on the physical vehicle relies on camera-based detection of orange and blue floor lines and red and green pillars.

### Why three separate simulation worlds instead of a parametric launch

The corridor width varies per straight section in a real competition round. We decided against a single parametric world because Gazebo SDF does not support runtime parameter substitution for model poses without a plugin. Three pre-built worlds cover the main test scenarios.

### Wall height in simulation 300 mm instead of 100 mm

We set wall height to 300 mm in all simulation worlds so that the LiDAR can detect them and SLAM Toolbox can build a map. This discrepancy between simulation and physical environment is a known limitation. The SLAM maps are used for reference and for validating the field geometry, not for runtime navigation on the physical vehicle.

---

## 10. Team Photos and Vehicle Photos

Photos to be added before final submission deadline.

- docs/photos/team_photo.jpg
- docs/photos/vehicle_front.jpg
- docs/photos/vehicle_back.jpg
- docs/photos/vehicle_top.jpg
- docs/photos/vehicle_side_left.jpg
- docs/photos/vehicle_side_right.jpg

---

## 11. Videos

Videos to be added before final submission deadline.

- Open Challenge demonstration: YouTube link to be added
- Obstacle Challenge demonstration: YouTube link to be added

Each video shows the vehicle driving autonomously for at least 30 seconds as required by WRO documentation rules.

---

## License

This repository is made public as required by WRO rules and will remain publicly accessible for at least 12 months after the competition.
