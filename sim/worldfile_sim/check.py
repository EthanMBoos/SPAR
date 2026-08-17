"""Sim self-check, the verify skill's check 3 (seconds, not minutes):

    docker exec worldfile-sim bash -lc 'cd /ws/sim && MUJOCO_GL=egl python3 -m worldfile_sim.check'

Imports every sim module, loads the world, resolves every Husky id, and
renders one frame per world camera.
Touches no port and no ROS graph, so it is safe while a sim is running.
"""

import os
import sys

import mujoco

from worldfile_sim import sim


def main():
    path = sim.world_path()
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    sim.resolve_ids(model)
    renderer = mujoco.Renderer(model, height=240, width=320)
    for cam in range(model.ncam):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, cam)
        renderer.update_scene(data, camera=name)
        renderer.render()
        print(f"[worldfile] check: camera '{name}' renders", flush=True)

    print(f"[worldfile] check ok: {os.path.basename(path)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
