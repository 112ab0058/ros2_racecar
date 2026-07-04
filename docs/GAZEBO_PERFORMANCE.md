# Gazebo Performance Notes

Updated: 2026-07-05

The WSL workflow should prioritize stable real-time simulation over pretty visuals.

## Current Defaults

- Official race launch: `race.launch.py`
- Gazebo mode: server-only by default, `gz sim -r -s`
- Physics step: `max_step_size = 0.005`
- Race launch does not start SLAM or Nav2.
- `line_detector.py` debug image publishing is disabled by default.
- `turtlebot3_burger_cam` is tracked inside this repo under `models/`.
- Camera: `320 x 240`, `10 Hz`, camera visualize disabled.
- LiDAR: `360` samples, `8 Hz`, visualize disabled.

## Why These Settings

The previous 3.2 m world used `max_step_size = 0.001`, which is heavier than needed for the 160 cm project field. A 5 ms step is a better starting point for WSL while still keeping robot motion smooth enough for line following and obstacle avoidance.

Server-only Gazebo avoids GUI/rendering cost. Use GUI only when inspecting the scene.

## Required Race Topics

Only bridge these in race mode:

```text
/cmd_vel
/scan
/clock
/camera/image_raw
/camera/camera_info
```

Do not start SLAM/Nav2 in `race.launch.py`.

## Performance Checklist

1. Start race mode:

```bash
ros2 launch wro2026_sim race.launch.py
```

2. Confirm topics:

```bash
ros2 topic hz /scan
ros2 topic hz /camera/image_raw
ros2 topic echo /clock --once
```

3. Confirm controller output:

```bash
ros2 topic echo /cmd_vel --once
```

4. If simulation is slow:

- Keep using server-only mode.
- Do not run RViz, Foxglove, SLAM, and Gazebo GUI at the same time.
- Lower camera debug load; keep `line_detector` debug disabled.
- Increase `max_step_size` to `0.01` only if controller stability remains acceptable.
- Reduce obstacle count while tuning line following.
- If CPU is still high, lower LiDAR from `8 Hz` to `5 Hz` before changing controller logic.

## GUI Mode

For visual inspection only, run Gazebo GUI manually with the same world:

```bash
gz sim -r ~/ros2_ws/src/wro2026_sim/worlds/wro2026_field.sdf
```

Do not use GUI mode for long tuning runs unless the machine can keep real-time factor near 1.0.
