"""Sim self-check, the verify skill's check 3 (seconds, not minutes):

    docker exec spar-sim bash -lc 'cd /ws/sim && MUJOCO_GL=egl python3 -m spar_sim.check'

Imports every sim module, loads the world, resolves every named id both
the sim and the PX4 link need, and renders one frame per camera.
Touches no port and no ROS graph, so it is safe while a sim is running.
"""

import os
import sys

import mujoco

from spar_sim import px4_link, sensors, sim


def main():
    path = sim.world_path()
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    sim.resolve_ids(model)
    px4_link.resolve_ids(model)

    renderer = mujoco.Renderer(model, height=sensors.Camera.HEIGHT,
                               width=sensors.Camera.WIDTH)
    for cam in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
        renderer.update_scene(data, camera=name)
        renderer.render()
        print(f"[spar] check: camera '{name}' renders", flush=True)

    print(f"[spar] check ok: {os.path.basename(path)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
