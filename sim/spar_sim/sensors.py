"""Sensor message builders: odom, TF, GPS, lidar, camera.

Everything reads MjData directly. MuJoCo world coordinates are ROS
coordinates (right-handed, Z-up), so no conversion ever touches the data
path. The hard-won conventions are called out inline.
"""

import math

import mujoco
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, LaserScan, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage

SCAN_RAYS = 720
SCAN_RATE = 15.0
SCAN_MIN_RANGE = 0.1
SCAN_MAX_RANGE = 25.0
GPS_RATE = 10.0
GPS_DATUM_LAT_DEG = 42.0
GPS_DATUM_LON_DEG = -71.0
GPS_STDDEV_M = 0.03
EARTH_RADIUS_M = 6378137.0


def stamp_of(t):
    return Time(sec=int(t), nanosec=int((t - math.floor(t)) * 1e9))


def base_yaw(data, body):
    """Yaw from the body's world quaternion (MuJoCo stores w x y z)."""
    qw, qx, qy, qz = data.xquat[body]
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def odom_and_tf(model, data, body, t):
    """Odometry conventions:
    - odom frame == world frame (drift-free odometry, robot spawns at origin)
    - orientation is yaw-only
    - twist is in the CHILD (base_link) frame via mj_objectVelocity with
      mjOBJ_XBODY and flg_local=1 -- the hard-won lesson; anything else
      feeds Nav2 sign-flipped feedback off the +x heading.
    """
    px, py, pz = data.xpos[body]
    yaw = base_yaw(data, body)
    vel = np.zeros(6)  # [ang(3), lin(3)] in the local frame
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, body, vel, 1)

    stamp = stamp_of(t)
    ozw, ozz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = px
    odom.pose.pose.position.y = py
    odom.pose.pose.orientation.z = ozz
    odom.pose.pose.orientation.w = ozw
    odom.twist.twist.linear.x = vel[3]
    odom.twist.twist.angular.z = vel[2]

    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = "odom"
    tf.child_frame_id = "base_link"
    tf.transform.translation.x = px
    tf.transform.translation.y = py
    tf.transform.translation.z = pz
    tf.transform.rotation.z = ozz
    tf.transform.rotation.w = ozw
    return odom, TFMessage(transforms=[tf])


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

    The datum is deliberately private to the simulated receiver. Consumers
    establish their own local origin from the fixes, just as a real ROS
    localization stack does, so no datum is duplicated in ROS config.
    """

    def __init__(self):
        self._rng = np.random.default_rng()

    def fix(self, data, base_body, t):
        east, north, up = data.xpos[base_body]
        east, north, up = (
            np.array([east, north, up])
            + self._rng.normal(0.0, GPS_STDDEV_M, size=3)
        )

        lat0 = math.radians(GPS_DATUM_LAT_DEG)
        msg = NavSatFix()
        msg.header.stamp = stamp_of(t)
        msg.header.frame_id = "base_link"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = GPS_DATUM_LAT_DEG + math.degrees(
            north / EARTH_RADIUS_M)
        msg.longitude = GPS_DATUM_LON_DEG + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(lat0)))
        msg.altitude = up
        variance = GPS_STDDEV_M * GPS_STDDEV_M
        msg.position_covariance = [
            variance, 0.0, 0.0,
            0.0, variance, 0.0,
            0.0, 0.0, variance,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return msg


class Lidar:
    """One revolution in the base plane: ray directions in the world frame
    offset by the base yaw, mj_multiRay against geom groups 0+3 excluding
    the robot's own body, inf where nothing hit."""

    def __init__(self):
        self._dirs = np.zeros(SCAN_RAYS * 3)
        self._geomids = np.zeros(SCAN_RAYS, dtype=np.int32)
        self._dists = np.zeros(SCAN_RAYS)
        self._groups = np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8)

    def scan(self, model, data, lidar_site, lidar_frame, base_body, t):
        origin = data.site_xpos[lidar_site].copy()
        yaw = base_yaw(data, base_body)
        inc = 2.0 * math.pi / SCAN_RAYS  # [-pi, pi), endpoint excluded
        angles = -math.pi + np.arange(SCAN_RAYS) * inc + yaw
        self._dirs[0::3] = np.cos(angles)
        self._dirs[1::3] = np.sin(angles)
        self._dirs[2::3] = 0.0
        mujoco.mj_multiRay(model, data, origin, self._dirs, self._groups, 1,
                           base_body, self._geomids, self._dists, None,
                           SCAN_RAYS, SCAN_MAX_RANGE)
        ranges = np.where(self._geomids >= 0, self._dists, np.inf)

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
            # The robots' collision geoms live in group 3, hidden by
            # default; without this the Husky renders wheelless.
            Camera._vis.geomgroup[3] = 1
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
