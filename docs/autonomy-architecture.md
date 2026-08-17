# Ground autonomy architecture

SPAR keeps the complete ground mission layer in the single `spar` package.
The behavior tree decides what the Husky should do; every movement is still a
standard Nav2 `NavigateToPose` action.

```text
mission/command ───────────────> MissionActive
battery/state ─────────────────> BatteryLow ─────> ReturnToDock
RGB-D -> red_barrel point ─────> BarrelSeen ─────> Inspect
generated navigation goals ──────────────────────> Rounds
                                                    |
                                             NavigateToPose
                                                    |
                                                   Nav2
```

The reactive priority order is:

1. Stay `Idle` until `mission/command` is `start`.
2. If battery is low or stale, preempt work and `ReturnToDock`.
3. If a fresh barrel point exists outside the inspection cooldown, `Inspect`.
4. Otherwise cycle through the generated goals as `Rounds`.
5. A `stop` command preempts any active Nav2 goal and returns to `Idle`.

Battery hysteresis enters the dock branch at 30% and does not resume rounds
until 90%. The simulator charges only when the localized base is within the
configured dock radius. The utility-depot demo uses the original dock pose
`(-4, -14)`.

`red_barrel_detector` thresholds the aligned RGB image in HSV, samples metric
depth, transforms the observation into `map`, and publishes
`/husky/perception/red_barrel` as `geometry_msgs/msg/PointStamped`. The
`Inspect` leaf creates a Nav2 stand-off pose facing that point.

Wheel encoders and IMU feed the local EKF, which publishes
`odom -> base_link`. GPS, IMU, and wheel odometry feed the global EKF through
`navsat_transform_node`, which publishes `map -> odom`. The world datum makes
MuJoCo coordinates, generated goals, the dock, detections, and ROS `map`
coordinates describe the same space.

The mission tree, leaves, detector, battery simulator, localization, and launch
files all live in `ros/src/spar`. There are no air/PX4 packages, separate
perception package, or cross-robot common layer.
