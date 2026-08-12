"""The sim: steps MuJoCo, publishes the robot's ROS topics, consumes
cmd_vel, and runs the PX4 lockstep link. Run it headless in the sim
container:

    MUJOCO_GL=egl WORLD=utility_depot_40_v2 python3 -m spar_sim.sim

Two threads: this main thread owns MjData (step, publish, pace); an rclpy
spin thread services the cmd_vel subscription and nothing else. The ROS
thread never dereferences MjData -- it writes (v, w, stamp) under a lock
and the physics loop reads whatever is there.
"""

import os
import socket
import struct
import sys
import threading
import time

import mujoco
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan, NavSatFix
from tf2_msgs.msg import TFMessage

from spar_sim import sensors
from spar_sim.georeference import from_mjcf
from spar_sim.husky import (
    KinematicDrive,
    WHEEL_NAMES,
)
from spar_sim.px4_link import Px4Link
from spar_sim.robot_config import scan_site

CLOCK_RATE = 50.0
SCAN_RATE = sensors.SCAN_RATE
CAM_RATE = 5.0
GPS_RATE = sensors.GPS_RATE
ENCODER_RATE = sensors.ENCODER_RATE
IMU_RATE = sensors.IMU_RATE
VIEWER_RATE = 30.0

CMD_TIMEOUT_SEC = 0.5


def world_path():
    world = os.environ.get("WORLD", "utility_depot_40_v2")
    return os.path.normpath(os.path.join(
        os.path.dirname(__file__), "..", "worlds", f"{world}.xml"))


def resolve_ids(model):
    """Fail loudly at startup, listing what was and wasn't found: a rename
    anywhere in the MJCF/python contract should die here, not five minutes
    into a smoke run."""
    obj = mujoco.mjtObj
    try:
        lidar_frame, lidar_id = scan_site(model, "husky")
    except ValueError as exc:
        sys.exit(f"[spar] sim could not resolve ids: {exc}")
    ids = {
        "base_link": mujoco.mj_name2id(model, obj.mjOBJ_BODY, "base_link"),
        "base_free": mujoco.mj_name2id(model, obj.mjOBJ_JOINT, "base_free"),
        "lidar": lidar_id,
        "lidar_frame": lidar_frame,
        "camera_link": mujoco.mj_name2id(model, obj.mjOBJ_SITE, "camera_0_link"),
        "gps": mujoco.mj_name2id(model, obj.mjOBJ_SITE, "gps_link"),
        "imu": mujoco.mj_name2id(model, obj.mjOBJ_SITE, "imu_link"),
        "footprint": mujoco.mj_name2id(
            model, obj.mjOBJ_GEOM, "husky_kinematic_footprint"),
        "camera_0": mujoco.mj_name2id(model, obj.mjOBJ_CAMERA, "camera_0"),
    }
    for position in WHEEL_NAMES:
        ids[f"{position}_joint"] = mujoco.mj_name2id(
            model, obj.mjOBJ_JOINT, f"{position}_wheel_joint")
    missing = [k for k, v in ids.items()
               if k != "lidar_frame" and v < 0]
    if missing:
        sys.exit(f"[spar] sim could not resolve ids: missing {missing}, "
                 f"resolved {ids}")
    print(
        f"[spar] sim ids ok: body={ids['base_link']} site={ids['lidar']} "
        "kinematic base and four wheel encoders resolved",
        flush=True,
    )
    return ids


class ViewerStream:
    """One-way qpos stream for the host viewer (sim/viewer.py): accepts one
    client on :9000, sends a magic + nq handshake so a world mismatch fails
    loudly, then time + qpos as little-endian float64s at ~30 Hz. View only
    by design; a viewer that can push on the world quietly breaks eval
    honesty."""

    MAGIC = b"SPAR"

    def __init__(self, model, port=9000):
        self._nq = model.nq
        self._conn = None
        self._lock = threading.Lock()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("0.0.0.0", port))
        self._listener.listen(1)
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while True:
            conn, _ = self._listener.accept()
            conn.settimeout(0.5)
            try:
                conn.sendall(self.MAGIC + struct.pack("<I", self._nq))
            except OSError:
                conn.close()
                continue
            with self._lock:
                if self._conn is not None:
                    self._conn.close()
                self._conn = conn

    def send(self, data):
        with self._lock:
            if self._conn is None:
                return
            try:
                self._conn.sendall(
                    struct.pack("<d", data.time) + data.qpos.tobytes())
            except OSError:  # viewer detached; the sim never notices
                self._conn.close()
                self._conn = None


def main():
    path = world_path()
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    ids = resolve_ids(model)
    georeference = from_mjcf(model)
    print(
        "[spar] world datum: "
        f"{georeference.latitude_deg:.7f}, "
        f"{georeference.longitude_deg:.7f}, "
        f"{georeference.altitude_m:.3f} m",
        flush=True,
    )

    rclpy.init()
    node = rclpy.create_node("spar_sim", namespace="husky")
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    encoder_pub = node.create_publisher(
        JointState, "platform/joint_states", 10)
    imu_pub = node.create_publisher(Imu, "sensors/imu/data", 10)
    scan_pub = node.create_publisher(LaserScan, "sensors/lidar2d_0/scan", 10)
    gps_pub = node.create_publisher(NavSatFix, "sensors/gps/fix", 10)
    static_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    tf_static_pub = node.create_publisher(TFMessage, "tf_static", static_qos)

    husky_cam = sensors.Camera(model, node, "camera_0", "sensors/camera_0")
    drone_cam = sensors.Camera(model, node, "x2_camera", "/skydio/sensors/camera_0")
    print("[spar] camera sensor mounted", flush=True)

    lidar = sensors.Lidar()
    gps = sensors.Gps(georeference)
    wheel_encoders = sensors.WheelEncoders([
        model.jnt_qposadr[ids[f"{position}_joint"]]
        for position in WHEEL_NAMES
    ])
    ground_drive = KinematicDrive(
        model,
        data,
        ids["base_free"],
        [ids[f"{position}_joint"] for position in WHEEL_NAMES],
        ids["footprint"],
    )
    ground_imu = sensors.GroundImu(model.jnt_dofadr[ids["base_free"]])
    px4 = Px4Link(model, georeference)
    viewer = ViewerStream(model)

    # cmd_vel state (written by the ROS thread, read in the physics loop).
    # cmd[2] is the sim time at receipt, cached for the ROS thread.
    lock = threading.Lock()
    cmd = [0.0, 0.0, -1.0]  # v, w, stamp
    sim_time = [0.0]

    def on_cmd_vel(msg: TwistStamped):
        with lock:
            cmd[0] = msg.twist.linear.x
            cmd[1] = msg.twist.angular.z
            cmd[2] = sim_time[0]

    node.create_subscription(TwistStamped, "cmd_vel", on_cmd_vel, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    tf_static_pub.publish(sensors.static_tf(model, [
        (ids["lidar_frame"], ids["lidar"]),
        ("camera_0_link", ids["camera_link"]),
        ("gps_link", ids["gps"]),
        ("imu_link", ids["imu"]),
    ]))

    next_clock = next_encoder = next_imu = 0.0
    next_scan = next_gps = next_cam = next_view = 0.0
    # Wall-clock pacing anchors: RTF 1.
    wall0, sim0 = time.monotonic(), data.time

    while rclpy.ok():
        with lock:
            v, w, stamp = cmd
        # stale command is no command
        if stamp < 0.0 or data.time - stamp > CMD_TIMEOUT_SEC:
            v = w = 0.0
        px4.step(model, data)
        mujoco.mj_step(model, data)
        ground_drive.advance(model, data, v, w, model.opt.timestep)
        t = data.time
        with lock:
            sim_time[0] = t

        if t >= next_clock:
            next_clock = t + 1.0 / CLOCK_RATE
            clock_pub.publish(Clock(clock=sensors.stamp_of(t)))
        if t >= next_encoder:
            next_encoder = t + 1.0 / ENCODER_RATE
            encoder_pub.publish(wheel_encoders.joint_state(data, t))
        if t >= next_imu:
            next_imu = t + 1.0 / IMU_RATE
            imu_pub.publish(ground_imu.sample(
                model,
                data,
                ids["base_link"],
                t,
                world_acceleration=ground_drive.world_acceleration,
            ))
        if t >= next_scan:
            next_scan = t + 1.0 / SCAN_RATE
            scan_pub.publish(lidar.scan(
                model, data, ids["lidar"], ids["lidar_frame"],
                ids["base_link"], t))
        if t >= next_gps:
            next_gps = t + 1.0 / GPS_RATE
            gps_pub.publish(gps.fix(data, ids["gps"], t))
        if t >= next_cam:
            next_cam = t + 1.0 / CAM_RATE
            husky_cam.publish(data, t)
            drone_cam.publish(data, t)
        if t >= next_view:
            next_view = t + 1.0 / VIEWER_RATE
            viewer.send(data)

        # Sleep the surplus. After a stall, re-anchor instead of bursting to
        # catch up: planners and vision run in wall time, and a free-running
        # sim breaks them exactly like time acceleration did.
        ahead = (t - sim0) - (time.monotonic() - wall0)
        if ahead > 0:
            time.sleep(ahead)
        elif ahead < -0.1:
            wall0, sim0 = time.monotonic(), t


if __name__ == "__main__":
    main()
