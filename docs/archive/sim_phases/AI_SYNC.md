> 已封存（2026-10-13）。目前狀態見 docs/STATUS.md。

# AI Sync

Last updated: 2026-07-06

## Current Task

Tune the 160 cm Gazebo race controller for a single obstacle at:

- `obstacle_yellow_mid`: `x=0.00 y=0.42`

Do not enable 2 or 3 obstacles. Do not add AI, Nav2, SLAM, or parking behavior.

## Current Code State

`src/wro2026_sim/scripts/line_follower.py` has uncommitted controller work:

- Three-phase avoidance states:
  - `AVOID_LANE_SHIFT`
  - `AVOID_PASS_OBSTACLE`
  - `AVOID_RETURN_HEADING`
- Return-to-line heading gate and timeout logging.
- Conservative LiDAR cluster trigger experiment:
  - ROI: `-60..+60 deg`
  - cluster threshold: `0.50 m`
  - obstacle max min-range: `0.215 m`
  - width gate: `2..42 deg`
  - point-count gate: `2..42`

Do not commit this state unless the regression tests pass.

## Latest Result

Partial.

The no-obstacle regression was re-run after explicitly verifying that
`obstacle_yellow_mid` was moved to the center holding pose:

- runtime `obstacle_yellow_mid`: `x=0.00 y=0.00`
- `is_obstacle_active=no` for the B-point obstacle

The 0.215 m cluster min-range gate still false-triggers in this clean
no-obstacle run. Evidence:

- `avoid_start_count=1`
- false trigger pose: `x=+0.135 y=+0.383 yaw=+3.141`
- false trigger front: `0.199`
- `recover=True`
- `recover_failed=False`

This confirms the previous failure was not caused by leaving the B-point
obstacle active. The 0.215 m min-range gate is not sufficient by itself.

## Next Direction

Do not keep raising or lowering one threshold blindly.

Next step should be a paired diagnostic, not another one-threshold tweak:

- disable or bypass the formal AVOID action for the diagnostic run only;
- record no-obstacle and B-point scans over the same pose band;
- compare top 3 clusters for `x=0.08..0.20`, `y=0.36..0.43`;
- only after that choose a new gate.

If paired scans still overlap, report that LiDAR-only cluster gating cannot
separate the B obstacle from normal top geometry at this pose.
