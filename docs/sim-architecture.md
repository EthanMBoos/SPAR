# The simulation architecture

One decision drives everything here: MuJoCo is the sim, full stop. One
Python process (`sim/spar_sim/`) steps the physics, renders the cameras
headless through EGL, computes the ROS topics directly from MuJoCo's
state, and publishes them with rclpy. It runs in its own container,
sharing a network namespace with the container that runs everything ROS:
Nav2, AMCL, the behavior tree. Same graph, localhost apart.

```
        ┌───────── autonomy container: Nav2 / AMCL / behavior tree ──────┐
        │                       pure ROS, no sim                         │
        └───────────────────────────▲────────────────────────────────────┘
                                    │ ROS topics (zenoh, shared netns)
        ┌───────────────────────────┴────────────────────────────────────┐
        │   THE SIM: python + mujoco (sim/spar_sim), headless in its     │
        │   container · the MJCF file IS the world (sim/worlds/)         │
        │   lint → rasterize reads the same file (make map)              │
        └────────────────────────────────────────────────────────────────┘
          run it three ways, same env, same sensors, same physics:
            • HEADLESS  make start_sim: the stack's sim, tests and smoke
            • WATCHED   make view: a native viewer window on the host,
                        synced to the running sim, read-only
            • RL        a training loop drives the env instead of Nav2
```

Why MuJoCo:

- MuJoCo's contacts are trustworthy for a mobile base.
- The whole sim is one readable Python package and three MJCF files; a
  student can see every line between the physics and the topics.
- An RL policy trains and deploys on the same physics ([rl.md](rl.md)).

## Worlds

The MJCF file is the world (`sim/worlds/<world>.xml`). You edit it
directly; the robots come in by `<include>` from `sim/robots/`. `make
map` runs the same file through a lint gate (geometry the robot can hit
but its lidar can't see, blocked docks and waypoints, floating or
interpenetrating props, a settle test) and rasterizes the map AMCL
localizes against, so the map can't drift from the world.

Keep collision geometry to primitives (box/sphere/capsule/cylinder) and
mark decorative meshes `contype="0" conaffinity="0"`. That split is what
lets visuals get as fancy as they want without ever touching physics,
lidar, or the map.

The robot looks like a Husky but is a differential drive under the hood:
two drive spheres plus frictionless casters, because a four-wheel
skid-steer with honest friction barely pivots. The tuning and the
reasoning live in comments in `sim/robots/husky.xml`, next to the values
they explain.

## Perception

The camera is real in every run: the sim renders color + depth
(offscreen EGL, no display), and the container's detector turns pixels
into a labeled point on `perception/detections` (message `Detection`:
header, label, map-frame point). The label is generic on purpose: a
small pretrained VLM or segmentation model can replace the HSV node and
report whatever classes it sees, one Detection per hit, without the
topic name changing; the behavior tree filters at the subscription for
the labels it cares about (`anomaly_label`), not just "anomaly". That,
not vision RL, is how vision models enter this stack ([rl.md](rl.md)).

## Topics

Everything lives under `/husky`, published by the sim in every run mode:

| Topic / TF | Type | Notes |
| --- | --- | --- |
| `/clock` | rosgraph_msgs/Clock | the sim owns time |
| `platform/odom`, TF `odom→base_link` | nav_msgs/Odometry | drift-free, twist in the child frame |
| TF `base_link→{lidar2d_0_laser, camera_0_link}` | static TF | published once, latched |
| `sensors/lidar2d_0/scan` | LaserScan | 720 rays at 15 Hz |
| `sensors/camera_0/color/image` (+ `camera_info`, `…/depth/image`) | Image | rendered frames, 10 Hz |
| `perception/detections` | spar/Detection (map) | the pixel detector's labeled hits; `bt_executive`'s `anomaly_label` param picks the one label it acts on |
| `cmd_vel` (subscribed) | TwistStamped | Nav2 → skid-steer mixer |

Transport is Zenoh over TCP; all containers share one network namespace,
so every node and PX4 reach each other on localhost.

## Settled: tried it, or weighed it, and closed it

- Unity as the render/authoring front end (the MuJoCo plugin in-process,
  a C# sensor layer, ROS-TCP into the container): built, shipped, then
  removed 2026. The reasons are
  [docs/future/dropping-unity.md](future/dropping-unity.md).
- Pure Unity/PhysX, and Unity as a render-only viewer over container
  physics: both rejected earlier for the same root cause, physics
  authority must not be split or dishonest.
- Time acceleration (RTF): deleted. Planners and vision don't speed up with
  sim time, so the stack runs at RTF ≈ 1.
- Out of scope: manipulation, learning from pixels, high-fidelity hydro/aero.

## Roadmap

- The RL harness: `sim/worlds/<world>.xml` → pure-MuJoCo Gym env → deploy
  the policy as a behavior node ([rl.md](rl.md) has the full design).
- Themed worlds and moving agents, generated in a separate scenario repo
  and imported as MJCF files into `sim/worlds/`.
- Marine. A domain is three swaps, not a fork: dynamics (a force module
  in the sim loop), sensors (`sim/spar_sim/sensors.py`), and planning
  (above the topic boundary). The 2D lidar + map + AMCL are the ground
  domain's module, not universal truth. The air track proved the shape:
  its dynamics is `sim/spar_sim/px4_link.py` applying rotor forces in
  the sim loop, its sensors are HIL messages computed from the same
  MjData, and its planner is PX4 + a second BT in its own container
  (`air/src/spar_air`), joined to the ground stack only through the
  shared zenoh graph. What the build taught lives as comments next to
  the code that earned each lesson, mostly the link and the air
  Dockerfile.

The bring-up traps (clock-rewind restart order, lockstep engagement, the
hidden geom group in rendering, and friends) are documented as comments
next to the code that handles each one, mostly the sim package and the
Makefile.
