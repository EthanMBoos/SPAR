"""The viewer: a native mujoco.viewer window on the host, synced to the
running sim over its qpos stream. Runs from the repo venv:

    make view          (sim/viewer.py --world utility_depot_40_v2)

Loads the same MJCF the sim loaded and copies time + qpos from the stream
into a passive viewer. View only, strictly one way: commands, teleop, and
perturbation stay out (the mission layer is scripts/mission.sh, and a
viewer that can push on the world quietly breaks eval honesty). Kill it
and reattach mid-run; the sim never notices.

For a world with no sim running, use the repository inspector:
    make inspect WORLD=utility_depot_40_v2
"""

import argparse
import os
import socket
import struct
import sys

import mujoco
import mujoco.viewer

MAGIC = b"SPAR"


def read_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument(
        "--world", default=os.environ.get("WORLD", "utility_depot_40_v2"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9000)
    args = ap.parse_args()

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "worlds", f"{args.world}.xml")
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)

    try:
        sock = socket.create_connection((args.host, args.port), timeout=2.0)
    except OSError:
        sys.exit(f"no sim on {args.host}:{args.port} — start it with `make start_sim`")
    header = read_exact(sock, 8)
    if header is None or header[:4] != MAGIC:
        sys.exit("that isn't the sim's stream (bad magic)")
    nq = struct.unpack("<I", header[4:])[0]
    if nq != model.nq:
        sys.exit(f"world mismatch: sim streams nq={nq}, "
                 f"{args.world}.xml has nq={model.nq} — same WORLD on both sides?")
    sock.settimeout(5.0)
    frame_len = 8 * (1 + nq)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # MuJoCo hides groups 3+ by default. Group 3 includes blank-world
        # obstacles (generated collision meshes stay hidden through alpha=0),
        # and group 4 is ground separated for the navigation lidar.
        viewer.opt.geomgroup[3] = 1
        viewer.opt.geomgroup[4] = 1
        viewer.sync()
        while viewer.is_running():
            frame = read_exact(sock, frame_len)
            if frame is None:
                print("sim closed the stream", file=sys.stderr)
                break
            values = struct.unpack(f"<{1 + nq}d", frame)
            data.time = values[0]
            data.qpos[:] = values[1:]
            mujoco.mj_forward(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()
