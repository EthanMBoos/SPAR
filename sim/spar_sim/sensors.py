"""Sensor message builders: wheel encoders, IMU, GPS, lidar, and camera.

Everything reads MjData directly. MuJoCo world coordinates are ROS
coordinates (right-handed ENU, Z-up). The simulator publishes measurements;
ground localization, odometry integration, and dynamic TF belong to ROS.
"""

import math

import mujoco
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import (
    CameraInfo,
    Image,
    Imu,
    JointState,
    LaserScan,
    NavSatFix,
    NavSatStatus,
)
from tf2_msgs.msg import TFMessage

from spar_sim.georeference import Georeference
from spar_sim.husky import quantize_wheel_angle
from spar_sim.raycast import NavigationRayFan

SCAN_RAYS = 720
SCAN_RATE = 15.0
SCAN_MIN_RANGE = 0.1
SCAN_MAX_RANGE = 25.0
GPS_RATE = 10.0
GPS_STDDEV_M = 0.03
ENCODER_RATE = 50.0
IMU_RATE = 100.0
IMU_ORIENTATION_STDDEV_RAD = 0.005
IMU_GYRO_STDDEV_RAD_S = 0.001
IMU_ACCEL_STDDEV_M_S2 = 0.01


def stamp_of(t):
    return Time(sec=int(t), nanosec=int((t - math.floor(t)) * 1e9))


def base_yaw(data, body):
    """Yaw from the body's world quaternion (MuJoCo stores w x y z)."""
    qw, qx, qy, qz = data.xquat[body]
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class WheelEncoders:
    """Quantized A200 wheel positions and tick-derived velocities."""

    NAMES = (
        "front_left_wheel_joint",
        "rear_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_right_wheel_joint",
    )

    def __init__(self, qpos_addresses):
        self._qpos = tuple(qpos_addresses)
        self._last_positions = None
        self._last_time = None

    @staticmethod
    def _quantize(angle):
        return quantize_wheel_angle(angle)

    def joint_state(self, data, t):
        positions = [self._quantize(data.qpos[address]) for address in self._qpos]
        if self._last_positions is None or t <= self._last_time:
            velocities = [0.0] * len(positions)
        else:
            dt = t - self._last_time
            velocities = [
                (position - previous) / dt
                for position, previous in zip(positions, self._last_positions)
            ]
        self._last_positions = positions
        self._last_time = t

        msg = JointState()
        msg.header.stamp = stamp_of(t)
        msg.name = list(self.NAMES)
        msg.position = positions
        msg.velocity = velocities
        return msg


class GroundImu:
    """Deterministic noisy ENU AHRS/IMU measurement for planar localization.

    The orientation is world-referenced, as a real outdoor localization stack
    requires from an AHRS/magnetometer-capable IMU. Linear acceleration is
    specific force: an upright stationary sensor reads +g on body Z.
    """

    def __init__(self, free_dof_address, seed=3):
        self._free_dof = free_dof_address
        self._rng = np.random.default_rng(seed)

    def sample(self, model, data, body, t, world_acceleration=None):
        yaw = base_yaw(data, body) + self._rng.normal(
            0.0, IMU_ORIENTATION_STDDEV_RAD)
        angular = np.zeros(6)
        mujoco.mj_objectVelocity(
            model, data, mujoco.mjtObj.mjOBJ_XBODY, body, angular, 1)

        world_acc = (
            data.qacc[self._free_dof:self._free_dof + 3]
            if world_acceleration is None
            else np.asarray(world_acceleration)
        )
        qw, qx, qy, qz = data.xquat[body]
        conjugate = np.array([qw, -qx, -qy, -qz])
        specific_force = np.zeros(3)
        mujoco.mju_rotVecQuat(
            specific_force,
            np.array([world_acc[0], world_acc[1], world_acc[2] + 9.80665]),
            conjugate,
        )

        msg = Imu()
        msg.header.stamp = stamp_of(t)
        msg.header.frame_id = "imu_link"
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        orientation_variance = IMU_ORIENTATION_STDDEV_RAD ** 2
        msg.orientation_covariance = [
            orientation_variance, 0.0, 0.0,
            0.0, orientation_variance, 0.0,
            0.0, 0.0, orientation_variance,
        ]
        gyro = angular[:3] + self._rng.normal(
            0.0, IMU_GYRO_STDDEV_RAD_S, size=3)
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        gyro_variance = IMU_GYRO_STDDEV_RAD_S ** 2
        msg.angular_velocity_covariance = [
            gyro_variance, 0.0, 0.0,
            0.0, gyro_variance, 0.0,
            0.0, 0.0, gyro_variance,
        ]
        acceleration = specific_force + self._rng.normal(
            0.0, IMU_ACCEL_STDDEV_M_S2, size=3)
        msg.linear_acceleration.x = acceleration[0]
        msg.linear_acceleration.y = acceleration[1]
        msg.linear_acceleration.z = acceleration[2]
        acceleration_variance = IMU_ACCEL_STDDEV_M_S2 ** 2
        msg.linear_acceleration_covariance = [
            acceleration_variance, 0.0, 0.0,
            0.0, acceleration_variance, 0.0,
            0.0, 0.0, acceleration_variance,
        ]
        return msg


def static_tf(model, site_names_ids):
    """The sensor sites, expressed in the base_link frame. Published once,
    latched (transient-local QoS); subscribers that join late still get
    them."""
    tfs = []
    for name, site in site_names_ids:
        tf = TransformStamped()
        tf.header.stamp = stamp_of(0.0)
        tf.header.frame_id = "base_link"
        tf.child_frame_id = name
        x, y, z = model.site_pos[site]
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.w = 1.0
        tfs.append(tf)
    return TFMessage(transforms=tfs)


class Gps:
    """RTK-grade GPS fix from the robot's world position.

    The datum comes from the loaded world and is also exported to autonomy.
    Consumers project fixes against that fixed datum, so GPS, ROS ``map``, and
    MuJoCo world coordinates describe the same physical locations.
    """

    def __init__(self, georeference: Georeference):
        self._georeference = georeference
        self._rng = np.random.default_rng(1)

    def fix(self, data, gps_site, t):
        east, north, up = data.site_xpos[gps_site]
        east, north, up = (
            np.array([east, north, up])
            + self._rng.normal(0.0, GPS_STDDEV_M, size=3)
        )

        latitude, longitude, altitude = self._georeference.enu_to_geodetic(
            east, north, up
        )
        msg = NavSatFix()
        msg.header.stamp = stamp_of(t)
        msg.header.frame_id = "gps_link"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        variance = GPS_STDDEV_M * GPS_STDDEV_M
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg


class Lidar:
    """Multi-channel lidar collapsed to the planar scan consumed by Nav2.

    Every horizontal bearing has a vertical fan of physical rays from the
    puck. The closest non-ground collision return becomes that bearing's
    horizontal range, allowing short pallets and curbs to enter the 2D
    costmap without pretending a single horizontal laser could see them.
    """

    def __init__(self):
        self._ray_fan = NavigationRayFan(SCAN_RAYS, SCAN_MAX_RANGE)

    def scan(self, model, data, lidar_site, lidar_frame, base_body, t):
        origin = data.site_xpos[lidar_site].copy()
        yaw = base_yaw(data, base_body)
        inc = 2.0 * math.pi / SCAN_RAYS  # [-pi, pi), endpoint excluded
        angles = -math.pi + np.arange(SCAN_RAYS) * inc + yaw
        ranges, _ = self._ray_fan.scan(
            model, data, origin, angles, base_body)

        msg = LaserScan()
        msg.header.stamp = stamp_of(t)
        msg.header.frame_id = lidar_frame
        msg.angle_min = -math.pi
        msg.angle_max = -math.pi + (SCAN_RAYS - 1) * inc
        msg.angle_increment = inc
        msg.scan_time = 1.0 / SCAN_RATE
        msg.range_min = SCAN_MIN_RANGE
        msg.range_max = SCAN_MAX_RANGE
        msg.ranges = ranges.astype(np.float32).tolist()
        return msg


class Camera:
    """Renders color + metric depth from an MJCF camera and publishes the
    three topics the pixel detector consumes: rgb8 color, 32FC1 eye-depth
    (meters), and camera_info with K = [fy 0 cx / 0 fy cy / 0 0 1].

    The header frame is the optical frame and is deliberately absent from
    TF; the detector's camera_frame parameter handles that (documented in
    anomaly_detector.cpp and the yaml). One Renderer is shared by every
    camera: creation is the expensive part, update_scene is not.
    """

    WIDTH, HEIGHT, FOVY_DEG = 320, 240, 58.0

    _renderer = None
    _vis = None

    @classmethod
    def _shared_renderer(cls, model):
        if Camera._renderer is None:
            Camera._renderer = mujoco.Renderer(model, height=cls.HEIGHT,
                                               width=cls.WIDTH)
            Camera._vis = mujoco.MjvOption()
            # Generated collision meshes and blank-world obstacles share group
            # 3. The generated meshes remain invisible through alpha=0, while
            # blank's visible obstacles need the group enabled. Ground is group
            # 4 so lidar can exclude it. MuJoCo hides both groups by default.
            Camera._vis.geomgroup[3] = 1
            Camera._vis.geomgroup[4] = 1
        return Camera._renderer, Camera._vis

    def __init__(self, model, node, camera_name, topic_prefix):
        self._name = camera_name
        self._renderer, self._visopt = self._shared_renderer(model)
        self._color_pub = node.create_publisher(
            Image, f"{topic_prefix}/color/image", 10)
        self._depth_pub = node.create_publisher(
            Image, f"{topic_prefix}/depth/image", 10)
        self._info_pub = node.create_publisher(
            CameraInfo, f"{topic_prefix}/color/camera_info", 10)

        fy = (self.HEIGHT / 2.0) / math.tan(math.radians(self.FOVY_DEG) / 2.0)
        self._info = CameraInfo()
        self._info.height = self.HEIGHT
        self._info.width = self.WIDTH
        self._info.k = [fy, 0.0, self.WIDTH / 2.0,
                        0.0, fy, self.HEIGHT / 2.0,
                        0.0, 0.0, 1.0]

    def publish(self, data, t):
        header_stamp = stamp_of(t)

        def image(encoding, step, payload):
            msg = Image()
            msg.header.stamp = header_stamp
            msg.header.frame_id = "camera_0_color_optical_frame"
            msg.height = self.HEIGHT
            msg.width = self.WIDTH
            msg.encoding = encoding
            msg.step = step
            msg.data = payload
            return msg

        r = self._renderer
        r.update_scene(data, camera=self._name, scene_option=self._visopt)
        rgb = r.render()  # HxWx3 uint8, top row first: publish as is
        self._color_pub.publish(image("rgb8", self.WIDTH * 3, rgb.tobytes()))

        r.enable_depth_rendering()
        depth = r.render()  # metric meters along the camera axis
        r.disable_depth_rendering()
        self._depth_pub.publish(
            image("32FC1", self.WIDTH * 4,
                  depth.astype(np.float32).tobytes()))

        self._info.header.stamp = header_stamp
        self._info.header.frame_id = "camera_0_color_optical_frame"
        self._info_pub.publish(self._info)
