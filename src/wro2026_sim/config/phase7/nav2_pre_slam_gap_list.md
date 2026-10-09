# Nav2 Pre-SLAM Gap List

## Current TurtleBot3 simulation

- Confirm the Phase 7C runtime topic/frame/rate inventory.
- Keep `map -> odom` absent during this inspection.
- During mapping, exactly one SLAM authority must provide `map -> odom`.
- During localization, AMCL may provide `map -> odom`; it must not run
  concurrently with a SLAM authority for the same transform.
- Validate scan timing, usable range, QoS, and frame connectivity before
  creating a costmap.
- Decide whether raw differential-drive odometry covariance is usable or
  whether a later `robot_localization` design is required.
- Produce and validate map resolution, origin, occupancy thresholds, footprint,
  inflation, and lifecycle/initial-pose contracts in a later phase.

## V550 Ackermann blockers

- Physical URDF/Xacro with axle, steering-pivot, wheel, body, and sensor frames.
- Measured wheelbase, track widths, steering limits, overhangs, tyre geometry,
  and minimum turning radius.
- Measured LiDAR, camera, and IMU extrinsics.
- Encoder resolution, steering feedback, timestamps, sign conventions, and
  covariance model.
- Hardware odometry source and `odom -> base_footprint` ownership.
- Ackermann command adapter and steering/drive safety limits.
- Static footprint validation plus swept-path and parking-entry validation.

No Ackermann feasibility claim is permitted until these inputs are measured.
