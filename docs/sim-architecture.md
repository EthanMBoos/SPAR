# Simulator architecture

One Python process owns MuJoCo state. It steps the world at 500 Hz, advances
the kinematic Husky from the latest collision-monitored `cmd_vel`, publishes
`/clock`, and emits lidar, GPS, IMU, wheel encoder, and aligned RGB-D messages
under `/husky`. The ROS callback thread only updates the latest command; the
physics thread is the sole owner of `MjData`.

The MJCF is authoritative for world geometry, collision meshes, timestep,
geographic datum, Husky spawn, sensor mounts, and viewer cameras. Generated
worlds attach the pose-neutral model in `sim/robots/husky.xml` at their
authored `husky_spawn` site.

The Husky base is intentionally kinematic. Wheel joints remain visible and
provide quantized encoder measurements, while a hidden footprint probe rejects
translations that overlap world collision geometry. Lidar casts a vertical
fan at each bearing so low obstacles enter Nav2's 2D costmaps.

The containers share the router's network namespace. This keeps ROS discovery,
the host viewer stream, and browser-backed RViz simple while leaving simulator
and navigation processes independently restartable. Start the simulator before
the ROS launch because the simulator owns `/clock` and a restart rewinds time.
