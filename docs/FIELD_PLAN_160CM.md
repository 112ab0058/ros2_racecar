# 160 x 160 cm Field Plan

Updated: 2026-07-05

The physical field is one fixed 1600 x 1600 mm area. It should not become many unrelated layouts. Instead, it uses movable modules inside one consistent field to test speed, line following, continuous obstacle avoidance, docking, and later navigation.

## Fixed Dimensions

| Item | Value |
|---|---|
| Field size | 1600 x 1600 mm |
| Central island | 400 x 400 mm |
| Nominal lane width | about 600 mm |
| Corner radius target | R80 mm visual/physical approximation |
| High-speed straight | about 1100 mm on the south side |
| Continuous obstacle zone | north side |
| Parking / docking zone | east side |
| Line width | 20 mm orange/blue tape |
| Obstacle modules | 40-60 mm footprint, 80-150 mm height |
| Wall height | 100-150 mm physical, 300 mm in Gazebo for LiDAR |

## Zone Purpose

| Zone | Purpose |
|---|---|
| South straight | Show speed advantage and speed-zone control |
| North obstacle section | Show continuous avoidance without stopping |
| Corners | Test orange/blue line trigger and yaw-based turn completion |
| East docking bay | Future parking/stop-at-station behavior |
| Central island | Creates a true loop and gives LiDAR stable wall references |

## Gazebo Coordinate Convention

The Gazebo field uses meters:

```text
field center: (0, 0)
outer boundary: x/y = -0.8 to +0.8
central island: x/y = -0.2 to +0.2
high-speed straight: y around -0.5
obstacle zone: y around +0.5
parking/docking bay: x around +0.6
```

## Physical Materials

Recommended first purchase/build list:

- Two 800 x 1600 mm base boards or one 1600 x 1600 mm board if transport is easy.
- Matte gray or matte white surface; avoid glossy material.
- Black wall strips, 100-150 mm tall.
- Green central island plate, 400 x 400 mm.
- Orange and blue matte tape, 20 mm wide.
- Yellow movable obstacle blocks.
- Blue parking zone tape or removable panel.
- Velcro, magnets, or locating holes so obstacles and parking dividers can be moved but repeatably placed.

## Current Implementation Choice

The first Gazebo version keeps the lines and uses the existing line detector. This lets the old controller continue to work while the new field and obstacle zone are introduced.

Do not generate a new map until the world dimensions and obstacle module positions are stable. The old 3.2 m map files were removed from the mainline to avoid using the wrong scale by accident.
