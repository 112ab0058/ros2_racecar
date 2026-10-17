# 多感測器融合感知與智慧決策系統

NTUT capstone project: a ROS 2 autonomous car on a **WHEELTEC R550 mini akm** (Ackermann steering, STM32 C30D controller, LSLIDAR N10P, WHEELTEC C70 camera) with a **Jetson Orin Nano** (Ubuntu 24.04, ROS 2 Jazzy). Due mid-November 2026.

- Current status: [docs/STATUS.md](docs/STATUS.md) (Traditional Chinese)
- Scope, architecture and plan: [docs/research/PROJECT_PLAN.md](docs/research/PROJECT_PLAN.md)
- Document index: [docs/README.md](docs/README.md)
- Rules for AI assistants: [CLAUDE.md](CLAUDE.md)

## Packages

| Package | What |
|---|---|
| `src/wro2026_base_r550` | R550 chassis: STM32 serial protocol (checked against the official WHEELTEC document and on the car), read-only feedback node (`/odom`, `/imu/data`, `/battery`), `listen` tool |
| `src/wro2026_lidar_n10p` | LSLIDAR N10P lidar driver in pure Python (read only, `/scan`); protocol taken from the official LSLIDAR driver |
| `src/wro2026_description` | URDF: measured sensor positions (TF tree under `base_link` = rear axle centre), `robot_state_publisher` launch |
| `src/wro2026_sim` | Finished Gazebo simulation line (2026-05 to 2026-07), kept as reference only |

The earlier transitional car (Waveshare WAVE ROVER) and its packages were removed on 2026-10-16; they remain in the git history.
