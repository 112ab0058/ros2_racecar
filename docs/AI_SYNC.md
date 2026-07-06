# AI Sync

Updated: 2026-07-06

This file is the compact source of truth for handing this project between ChatGPT/GPT and Codex.

## Current Repositories

| Item | Value |
|---|---|
| Git repo | `~/ros2_racecar` |
| Runtime workspace | `~/ros2_ws` |
| Main package | `src/wro2026_sim` |
| GitHub main | `5f65dbc Tune return-to-line exit for one-obstacle test` |
| WSL local state | May be `main...origin/main [ahead 2]` because WSL cannot push and has local tuning commits/changes |

Do not assume WSL local commits are already on GitHub. If WSL has useful work, export a single-commit patch with `git format-patch -1 <commit> --stdout`.

## Stable Project Facts

- Field is a fixed `1600 x 1600 mm` Gazebo/physical planning layout.
- Official world is `src/wro2026_sim/worlds/wro2026_field.sdf`.
- Orange/blue line marks must stay diagonal, copied from the old WRO-style corner trigger layout and scaled to 160 cm.
- Parking/docking divider models are visual-only in race mode so they do not block the course.
- `race.launch.py` must stay lightweight and must not start SLAM or Nav2.
- GUI is optional through `gui:=true`; server-only remains the default.
- Use system Python for ROS 2 Jazzy: `/usr/bin/python3`, Python `3.12.x`.

## Current Behavior Status

| Test | Status | Evidence |
|---|---|---|
| Gazebo launch | Done | `race.launch.py` runs, GUI can attach, topics are present |
| No-obstacle baseline | Done | 3 laps completed, no `RECOVER` / `RECOVER_FAILED`, stable blue-trigger turns |
| 1 obstacle return-to-line | Done | `AVOID_OBSTACLE` and `RETURN_TO_LINE` occurred; `timeout_clear` returned to `STRAIGHT` |
| 1 obstacle full lap | Partial | B position can sometimes return bottom, but current trigger/avoid behavior is not robust |
| 2/3 obstacles | Not started | Do not start until 1 obstacle is stable |
| Parking/docking | Not started | Keep visual-only for now |
| Nav2/SLAM race flow | Not started | Keep out of `race.launch.py` |

## Current 1-Obstacle Diagnosis

Obstacle positions tested:

| Position | Pose | Result |
|---|---|---|
| A | `x=-0.12 y=0.42` | Partial, caused `RECOVER_FAILED` |
| B | `x=0.00 y=0.42` | Best current 1-obstacle test position |
| C | `x=+0.12 y=0.42` | Partial, too close to upper-lane entry |

Use B for the next 1-obstacle work:

```text
obstacle_yellow_mid x=0.00 y=0.42 z=0.075
```

The latest diagnosis found that no-obstacle upper geometry can look similar to early B-obstacle readings if the controller only checks sector values. The next technical step is to distinguish a narrow obstacle cluster from long wall/island geometry in the LiDAR scan.

## Current Controller Direction

The controller is still based on:

```text
/camera/image_raw -> line_detector.py -> /line_detection
/scan + /odom + /line_detection -> line_follower.py -> /cmd_vel
```

Keep this architecture. Do not remove the line detector.

The current obstacle logic has been moving toward:

- startup line lockout
- LiDAR obstacle trigger in the upper zone
- staged avoidance phases
- `RETURN_TO_LINE` timeout protections

The next change should not be a larger state explosion. It should improve obstacle trigger classification using scan cluster shape.

## Next Recommended Task

Implement a LiDAR scan cluster gate for obstacle trigger:

- Keep no-obstacle upper pass at `avoid_start_count=0`.
- Allow B obstacle to trigger earlier than the current late trigger.
- Distinguish narrow obstacle cluster from normal field geometry.
- Do not start 2/3 obstacle tests yet.

See `docs/NEXT_PROMPT.md` for the exact next Codex prompt.

