# Test Evidence

Updated: 2026-07-06

This file records high-signal evidence from Gazebo/ROS tests. It is intentionally concise so ChatGPT/GPT can review progress without reading the whole chat history.

## Evidence Requirements

Each test round should include:

- Git status and diff stat
- Python and build evidence
- ROS topic/probe evidence
- behavior logs
- final status: `Done`, `Partial`, or `Blocked`

## Test History

| Date | Test | Status | Key Evidence | Next Action |
|---|---|---|---|---|
| 2026-07-05 | Gazebo launch and GUI | Done | `race.launch.py` runs; `gz sim -g` can attach; topics include `/clock`, `/scan`, `/camera/image_raw`, `/cmd_vel`, `/line_detection`, `/odom`, `/tf` | Continue behavior tests |
| 2026-07-05 | No-obstacle baseline | Done | 3 laps, no `RECOVER` / `RECOVER_FAILED`, stable blue-trigger turns, bottom speed reached `0.180` | Start 1-obstacle testing |
| 2026-07-05 | 1 obstacle initial return-to-line | Partial | Entered `AVOID_OBSTACLE` and `RETURN_TO_LINE`, but got stuck because `RETURN_TO_LINE_DIST=0.14` was too strict | Add return-to-line timeout and smaller distance |
| 2026-07-05 | 1 obstacle return-to-line fix | Done | `RETURN_TO_LINE exit reason=timeout_clear elapsed=2.50s`, returned to `STRAIGHT`, no `RECOVER_FAILED` | Test complete 1-obstacle lap |
| 2026-07-06 | 1 obstacle full-lap attempt | Partial | Three-phase avoidance ran, but did not leave upper zone reliably; no `RECOVER_FAILED` | Diagnose geometry and trigger |
| 2026-07-06 | Geometry scan A/B/C | Partial / diagnosis complete | B `x=0.00 y=0.42` is best; A caused `RECOVER_FAILED`; C is too close to entry; no-obstacle geometry can resemble early obstacle readings | Implement scan cluster trigger gate |
| 2026-07-06 | Sector gate for trigger | Partial | No-obstacle false trigger fixed, but B obstacle triggered too late because gate required `near_front` | Use cluster shape instead of sector-only gate |

## Known Good Evidence

No-obstacle 3-lap baseline:

```text
No RECOVER / RECOVER_FAILED.
Blue trigger left turns are stable.
Bottom fast zone speed reaches 0.180.
Bottom return poses:
Lap 1: x=-0.114 y=-0.376 yaw≈0
Lap 2: x=-0.113 y=-0.399 yaw≈0
Lap 3: x=-0.098 y=-0.381 yaw≈0
```

1-obstacle return-to-line fix:

```text
AVOID_OBSTACLE start front≈0.338
RETURN_TO_LINE exit reason=timeout_clear elapsed=2.50s
Returned to STRAIGHT
No RECOVER_FAILED
front min≈0.320m
```

Geometry diagnosis:

```text
Obstacle B at x=0.00 y=0.42 is the best current test point.
A at x=-0.12 y=0.42 caused RECOVER_FAILED.
C at x=+0.12 y=0.42 starts too close to the obstacle.
No-obstacle upper geometry can produce diag_near readings, so sector-only trigger logic is insufficient.
```

## Current Risk

The controller can confuse normal upper-course geometry with a small obstacle if it only uses a few LiDAR sector distances. The next implementation should classify scan cluster shape to distinguish narrow obstacle clusters from wall/island geometry.

