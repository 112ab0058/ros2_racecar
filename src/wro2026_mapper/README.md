# wro2026_mapper

Independent WRO 2026 lawn-mower mapping tool.

This package provides the `wro2026_mapper` executable. It subscribes to `/odom`
and `/scan`, then publishes `/cmd_vel` to run a fixed coverage pattern.

Do not run this tool at the same time as
`wro2026_sim/scripts/line_follower.py`, because both nodes can publish
`/cmd_vel`.

Step 6/7 official line-triggered race control still uses:

```bash
ros2 launch ~/ros2_ws/src/wro2026_sim/launch/wro2026_step6_lidar.launch.py
```

Run this mapper tool separately:

```bash
ros2 run wro2026_mapper wro2026_mapper
```
