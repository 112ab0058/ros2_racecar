# V550 Geometry Measurement Template

Do not enter estimates. Record the physical datum, instrument, uncertainty,
measurement date, and responsible person for every value.

| Required input | Value | Unit | Datum / method | Uncertainty | Status |
|---|---:|---|---|---:|---|
| Wheelbase | — | m | Rear axle centre to front steering-pivot axle | — | BLOCKED |
| Front track width | — | m | Left/right tyre contact-centre spacing | — | BLOCKED |
| Rear track width | — | m | Left/right tyre contact-centre spacing | — | BLOCKED |
| Front tyre diameter | — | m | Loaded rolling diameter | — | BLOCKED |
| Rear tyre diameter | — | m | Loaded rolling diameter | — | BLOCKED |
| Front tyre width | — | m | Maximum physical width | — | BLOCKED |
| Rear tyre width | — | m | Maximum physical width | — | BLOCKED |
| Left steering pivot XYZ | — | m | Relative to proposed `base_link` | — | BLOCKED |
| Right steering pivot XYZ | — | m | Relative to proposed `base_link` | — | BLOCKED |
| Maximum inner steering angle | — | rad | Both turn directions under load | — | BLOCKED |
| Maximum outer steering angle | — | rad | Both turn directions under load | — | BLOCKED |
| Front overhang | — | m | Front axle to forward envelope | — | BLOCKED |
| Rear overhang | — | m | Rear axle to rear envelope | — | BLOCKED |
| Left body overhang | — | m | Vehicle datum to left envelope | — | BLOCKED |
| Right body overhang | — | m | Vehicle datum to right envelope | — | BLOCKED |
| LiDAR XYZ/RPY | — | m/rad | Relative to `base_link` | — | BLOCKED |
| Camera XYZ/RPY | — | m/rad | Relative to `base_link` | — | BLOCKED |
| IMU XYZ/RPY | — | m/rad | Relative to `base_link` | — | BLOCKED |
| `base_link` origin definition | — | — | Physical datum and axis convention | — | BLOCKED |
| Encoder resolution | — | counts/rev | At wheel after gearing | — | BLOCKED |
| Steering feedback | — | — | Sensor, units, sign, rate, limits | — | BLOCKED |

Also record drive-command transport, encoder/steering timestamps, covariance
model, sensor time synchronization, and the measured minimum turning circle in
both directions.
