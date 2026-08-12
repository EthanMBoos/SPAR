#!/usr/bin/env python3
"""Open one world directly in the native MuJoCo viewer."""

import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--world", default="utility_depot_40_v2")
    args = parser.parse_args()
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "worlds",
        f"{args.world}.xml",
    )
    if not os.path.isfile(path):
        sys.exit(
            f"world '{args.world}' does not exist at {path}\n"
            "export it first with worldgen/export.py")

    # Compile on this thread so missing assets and invalid MJCF retain their
    # useful MuJoCo error instead of becoming the viewer's "unknown exception".
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Match sim/viewer.py's proven macOS path. The managed viewer used by
    # `python -m mujoco.viewer` initializes a different AppKit/GLFW path.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.geomgroup[3] = 1
        viewer.opt.geomgroup[4] = 1
        viewer.sync()
        while viewer.is_running():
            time.sleep(0.05)


if __name__ == "__main__":
    main()
