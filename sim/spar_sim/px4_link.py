"""The PX4 HIL lockstep link, ported line for line from SparPx4Link.cs.

Per sensor tick it sends HIL_SENSOR (IMU, mag, baro) and HIL_GPS computed
from MjData, blocks for PX4's HIL_ACTUATOR_CONTROLS reply (lockstep), and
applies the four rotor forces at the thrust sites with mj_applyFT. One
socket, one loop, no MAVLink library: only four message types cross this
link, so the framing is hand rolled below.

Frames, decided once:
 - MuJoCo world is right-handed Z-up; this file treats +X east, +Y north.
   PX4 wants NED world and FRD body, so: north=+Y, east=+X, down=-Z, and
   body FLU -> FRD is (x, -y, -z). The matching NED->ENU / FRD->FLU halves
   live in the container's tf_from_px4.cpp, nowhere else.
 - HIL timestamps are MuJoCo sim time, so PX4's clock equals /clock by
   construction; a sim restart rewinds both, which is why the restart
   order in the Makefile header includes the air container.

PX4 v1.17's simulator_mavlink is the TCP CLIENT (px4-rc.mavlinksim starts
it with -c): this link listens on :4560 and PX4 dials out. The sim runs
inside core's network namespace, so the air container starts PX4 with
PX4_SIM_HOSTNAME=localhost and no port is ever published.
"""

import math
import random
import select
import socket
import struct
import sys
import time

import mujoco
import numpy as np

from spar_sim.georeference import Georeference

LISTEN_PORT = 4560
# Lockstep receive timeout. On timeout the rotors zero (the drone falls
# visibly instead of flying on stale commands), the link drops, and PX4
# needs a restart.
TIMEOUT_SEC = 0.1

# The rotors are a force module, not actuators: the X2's motors act at
# sites, and each step the link applies thrust +z and the drag torque
# km*thrust about z (both body frame) at each site via mj_applyFT.
# PX4 outputs follow none_iris rotor geometry (CA_ROTOR0..3: front-right,
# rear-left, front-left, rear-right). Sites listed in that PX4 order,
# matched to the Menagerie X2 by position. The km signs are the
# controller's, not the vendor's: CA_ROTOR_KM is defined in FRD (z DOWN),
# so +KM there is a NEGATIVE z-up torque here. Pairing the Menagerie gear
# signs naively inverts every yaw reaction and the drone spins up and
# crashes seconds after a clean takeoff. These are wire facts, not tuning
# knobs.
SITE_NAMES = ("thrust4", "thrust2", "thrust3", "thrust1")
KM_PER_FORCE = (-0.0201, -0.0201, 0.0201, 0.0201)
MAX_THRUST_N = 13.0  # Menagerie X2 motor ctrlrange upper bound

# The HIL receiver reports these metre-scale uncertainties to PX4. The
# simulator injects Gaussian errors with the same standard deviations.
GPS_HORIZONTAL_STDDEV_M = 0.30
GPS_VERTICAL_STDDEV_M = 0.60
GPS_VELOCITY_STDDEV_M_S = 0.25

# PX4 SITL's simulation contract is a 250 Hz sensor stream (it replies one
# HIL_ACTUATOR_CONTROLS per HIL_SENSOR at that rate; faster sends get
# every-other replies and the lockstep read times out). MuJoCo keeps
# stepping at 500 Hz; only the exchange is paced.
SENSOR_INTERVAL = 0.004

# Wire facts from the pinned PX4 build's generated headers
# (build/px4_sitl_zenoh/mavlink/): message id, v1 payload length (extension
# fields dropped -- the receiver zero-fills them), CRC_EXTRA.
MSG_HEARTBEAT, CRC_HEARTBEAT, LEN_HEARTBEAT = 0, 50, 9
MSG_HIL_SENSOR, CRC_HIL_SENSOR, LEN_HIL_SENSOR = 107, 108, 64
MSG_HIL_GPS, CRC_HIL_GPS, LEN_HIL_GPS = 113, 124, 36
MSG_HIL_ACTUATORS = 93


class GpsNoise:
    """Deterministic white-noise receiver model, independent of IMU draws."""

    def __init__(self, seed=2):
        self._rng = random.Random(seed)

    def sample(self, position_enu, velocity_ned):
        east, north, up = position_enu
        north_velocity, east_velocity, down_velocity = velocity_ned
        return (
            (
                east + self._rng.gauss(0.0, GPS_HORIZONTAL_STDDEV_M),
                north + self._rng.gauss(0.0, GPS_HORIZONTAL_STDDEV_M),
                up + self._rng.gauss(0.0, GPS_VERTICAL_STDDEV_M),
            ),
            (
                north_velocity
                + self._rng.gauss(0.0, GPS_VELOCITY_STDDEV_M_S),
                east_velocity
                + self._rng.gauss(0.0, GPS_VELOCITY_STDDEV_M_S),
                down_velocity
                + self._rng.gauss(0.0, GPS_VELOCITY_STDDEV_M_S),
            ),
        )


def resolve_ids(model):
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "x2")
    sites = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)
             for n in SITE_NAMES]
    if body < 0 or min(sites) < 0:
        sys.exit(f"[spar] px4 link could not resolve ids: body={body} "
                 f"sites={sites}")
    return body, sites


def _crc_accumulate(b, crc):
    tmp = (b ^ (crc & 0xFF)) & 0xFF
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


class Px4Link:

    def __init__(self, model, georeference: Georeference, port=LISTEN_PORT):
        self._body, self._sites = resolve_ids(model)
        self._georeference = georeference
        # The freejoint's first dof: qacc linear accel, world frame.
        self._free_dof = model.jnt_dofadr[model.body_jntadr[self._body]]
        print(f"[spar] px4 link ids ok: body={self._body} "
              f"sites={self._sites}", flush=True)

        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("0.0.0.0", port))
        self._listener.listen(1)
        self._listener.setblocking(False)

        self._sock = None
        self._first_reply = True
        self._reads = 0  # successful lockstep reads, for drop diagnostics
        self._seq = 0
        self._thrust_n = [0.0] * 4
        self._next_sensor = self._next_gps = self._next_heartbeat = 0.0
        # A noiseless sensor is a broken sensor to PX4: bit-identical
        # readings trip its stuck-sensor detection (the mag goes stale in
        # seconds and EKF never yaw-aligns), so every HIL simulator dithers
        # its outputs.
        self._noise = random.Random(1)
        self._gps_noise = GpsNoise()

    # ----------------------------------------------------------- lifecycle

    def _try_accept(self):
        try:
            sock, _ = self._listener.accept()
        except (BlockingIOError, OSError):
            return
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Nagle
        # would batch the per-step sends: no lockstep
        sock.settimeout(1.0)  # a partial frame must never hang the physics loop
        self._sock = sock
        self._first_reply = True
        print("[spar] px4 link connected", flush=True)

    def _drop(self, why):
        print(f"[spar] px4 link dropped ({why}, after {self._reads} lockstep "
              "reads); rotors zeroed, restart the air container to fly again",
              flush=True)
        self._reads = 0
        if self._sock is not None:
            self._sock.close()
        self._sock = None

    # ----------------------------------------------------------- lockstep

    def step(self, model, data):
        """Called by the sim loop before each mj_step. The drone owns
        qfrc_applied: cleared every step, then the rotor forces for this
        step (zero while disconnected, so the drone falls)."""
        t = data.time
        data.qfrc_applied[:] = 0.0
        if self._sock is None:
            self._try_accept()
            if self._sock is None:
                return
        try:
            if t >= self._next_sensor:
                self._next_sensor = t + SENSOR_INTERVAL
                if t >= self._next_heartbeat:
                    self._next_heartbeat = t + 1.0
                    self._send_heartbeat()
                if t >= self._next_gps:
                    self._next_gps = t + 0.1
                    self._send_hil_gps(model, data, t)
                self._send_hil_sensor(model, data, t)
                # PX4's clock only advances on HIL_SENSOR timestamps, and it
                # boots (sensors, EKF, commander) across many steps before
                # the first controls message. Blocking for a reply from step
                # one would deadlock both sides, so the link free-runs until
                # PX4's first HIL_ACTUATOR_CONTROLS, then locks step for
                # good.
                if self._first_reply:
                    self._poll_actuators()
                elif not self._read_actuators():
                    self._drop("lockstep timeout")
                    return
            self._apply_rotor_forces(model, data)
        except OSError as e:
            self._drop(type(e).__name__)

    def _apply_rotor_forces(self, model, data):
        quat = data.xquat[self._body]
        force = np.zeros(3)
        torque = np.zeros(3)
        for i in range(4):
            f = self._thrust_n[i]
            # Thrust +z and drag torque km*f about z, both body frame -> world.
            mujoco.mju_rotVecQuat(force, np.array([0.0, 0.0, f]), quat)
            mujoco.mju_rotVecQuat(
                torque, np.array([0.0, 0.0, KM_PER_FORCE[i] * f]), quat)
            point = data.site_xpos[self._sites[i]]
            mujoco.mj_applyFT(model, data, force, torque, point, self._body,
                              data.qfrc_applied)

    # Block until PX4's HIL_ACTUATOR_CONTROLS for this step arrives. PX4
    # sends exactly one per sensor step once lockstep engages
    # (SimulatorMavlink.cpp: lockstep progress on HIL_SENSOR, then
    # send_controls()), plus occasional heartbeats etc., which are skipped.
    # PX4 replies v2-framed (0xFD) with trailing-zero payload truncation.

    def _poll_actuators(self):
        while select.select([self._sock], [], [], 0)[0]:
            if self._parse_frame():
                self._first_reply = False
                print("[spar] px4 lockstep engaged", flush=True)
                return

    def _read_actuators(self):
        deadline = time.monotonic() + TIMEOUT_SEC
        self._sock.settimeout(TIMEOUT_SEC)
        while time.monotonic() < deadline:
            if self._parse_frame():
                self._reads += 1
                return True
        return False

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise OSError("connection closed")
            buf += chunk
        return bytes(buf)

    def _parse_frame(self):
        """Reads one frame; True when it was HIL_ACTUATOR_CONTROLS (thrusts
        stored). A recv timeout raises, which step() treats as a drop."""
        magic = self._read_exact(1)[0]
        if magic == 0xFD:            # v2: len incompat compat seq sys comp id[3]
            hdr = self._read_exact(9)
            payload_len = hdr[0]
            msg_id = hdr[6] | (hdr[7] << 8) | (hdr[8] << 16)
        elif magic == 0xFE:          # v1: len seq sys comp id
            hdr = self._read_exact(5)
            payload_len = hdr[0]
            msg_id = hdr[4]
        else:
            return False  # resync on the next magic byte
        payload = self._read_exact(payload_len + 2)[:payload_len]
        if msg_id != MSG_HIL_ACTUATORS:
            return False
        # controls[16] floats at offset 16 (after time_usec u64 + flags u64),
        # zero-extended when truncated; motors are 0..1 normalized.
        for i in range(4):
            off = 16 + 4 * i
            c = (struct.unpack_from("<f", payload, off)[0]
                 if off + 4 <= payload_len else 0.0)
            self._thrust_n[i] = min(max(c, 0.0), 1.0) * MAX_THRUST_N
        return True

    # ------------------------------------------------------------ sensors

    def _dither(self, sigma):
        return self._noise.gauss(0.0, sigma)

    def _send_hil_sensor(self, model, data, t):
        # The accelerometer must read specific force, f = R^T (a - g), which
        # is -g rotated into the body at rest, not zero. The freejoint's
        # translational qacc from the last completed step IS the body's
        # linear acceleration in world coordinates: exact, no finite
        # differencing (an aliased hand-rolled derivative here diverges EKF2
        # in flight).
        acc = data.qacc[self._free_dof:self._free_dof + 3]

        # Body-frame (FLU) angular velocity straight from MuJoCo.
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                 self._body, vel, 1)

        # World -> body via the conjugate of xquat (w x y z, body -> world).
        qw, qx, qy, qz = data.xquat[self._body]
        conj = np.array([qw, -qx, -qy, -qz])
        fb = np.zeros(3)
        mujoco.mju_rotVecQuat(
            fb, np.array([acc[0], acc[1], acc[2] + 9.80665]), conj)

        # Mid-latitude magnetic field, NED (gauss) -> world ENU -> body.
        mag_n, mag_d = 0.21523, 0.42741
        mb = np.zeros(3)
        mujoco.mju_rotVecQuat(mb, np.array([0.0, mag_n, -mag_d]), conj)

        # ISA barometer from geometric altitude.
        alt = (data.xpos[self._body][2]
               + self._georeference.altitude_m)
        pressure_hpa = 1013.25 * math.pow(1.0 - 2.25577e-5 * alt, 5.2559)

        payload = struct.pack(
            "<Q13fI",
            int(t * 1e6),
            # FLU -> FRD: keep x, negate y and z.
            fb[0] + self._dither(0.01),
            -fb[1] + self._dither(0.01),
            -fb[2] + self._dither(0.01),
            vel[0] + self._dither(0.001),
            -vel[1] + self._dither(0.001),
            -vel[2] + self._dither(0.001),
            mb[0] + self._dither(0.002),
            -mb[1] + self._dither(0.002),
            -mb[2] + self._dither(0.002),
            pressure_hpa + self._dither(0.02),
            0.0,
            alt,
            20.0,
            0x1FFF)  # all fields valid
        self._send(MSG_HIL_SENSOR, CRC_HIL_SENSOR, payload)

    def _send_hil_gps(self, model, data, t):
        east, north, up = data.xpos[self._body]
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                 self._body, vel, 0)
        (east, north, up), (vn, ve, vd) = self._gps_noise.sample(
            (east, north, up), (vel[4], vel[3], -vel[5])
        )
        lat, lon, alt = self._georeference.enu_to_geodetic(east, north, up)

        payload = struct.pack(
            "<QiiiHHHhhhHBB",
            int(t * 1e6),
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000.0),
            int(GPS_HORIZONTAL_STDDEV_M * 100.0),
            int(GPS_VERTICAL_STDDEV_M * 100.0),
            int(math.sqrt(vn * vn + ve * ve) * 100.0),
            int(vn * 100), int(ve * 100), int(vd * 100),
            0xFFFF,  # course over ground: unknown
            3, 12)   # 3D fix, 12 sats
        self._send(MSG_HIL_GPS, CRC_HIL_GPS, payload)

    def _send_heartbeat(self):
        # type MAV_TYPE_GCS keeps PX4 from treating us as a vehicle;
        # autopilot MAV_AUTOPILOT_INVALID.
        payload = struct.pack("<IBBBBB", 0, 6, 8, 0, 0, 3)
        self._send(MSG_HEARTBEAT, CRC_HEARTBEAT, payload)

    # ----------------------------------------------- MAVLink v1 framing
    # STX len seq sys comp msgid payload crc16, X25 CRC over len..payload
    # plus the per-message CRC_EXTRA byte.

    def _send(self, msg_id, crc_extra, payload):
        frame = bytearray([0xFE, len(payload), self._seq & 0xFF,
                           1,     # sysid: the simulated vehicle
                           200,   # compid: not the autopilot
                           msg_id])
        self._seq += 1
        frame += payload
        crc = 0xFFFF
        for b in frame[1:]:
            crc = _crc_accumulate(b, crc)
        crc = _crc_accumulate(crc_extra, crc)
        frame += bytes([crc & 0xFF, crc >> 8])
        self._sock.sendall(frame)
