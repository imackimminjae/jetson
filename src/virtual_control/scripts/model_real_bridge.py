#!/usr/bin/env python3
"""Align physical Motive motion to a scaled KNU map and relay Isaac's BEV."""
from copy import deepcopy
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from knu_global_path_publisher import load_start_positions


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ModelAlignment:
    """A physical metre stays a metre; only the virtual map anchor is scaled."""

    def __init__(self, anchor, target_yaw, scale):
        if (not math.isfinite(scale) or scale <= 0.0
                or not all(math.isfinite(value) for value in (*anchor, target_yaw))):
            raise ValueError('map scale and route anchor must be finite; scale > 0')
        self.anchor = tuple(value * scale for value in anchor)
        self.target_yaw = target_yaw
        self.scale = scale
        self.origin = None

    def transform(self, x, y, z, yaw):
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            raise ValueError('non-finite Motive pose')
        if self.origin is None:
            self.origin = (x, y, z, yaw)
        ox, oy, oz, oyaw = self.origin
        angle = wrap(self.target_yaw - oyaw)
        c, s = math.cos(angle), math.sin(angle)
        physical = (
            self.anchor[0] + c * (x - ox) - s * (y - oy),
            self.anchor[1] + s * (x - ox) + c * (y - oy),
            self.anchor[2] + z - oz,
            wrap(self.target_yaw + yaw - oyaw),
        )
        virtual = tuple(value / self.scale for value in physical[:3]) + (physical[3],)
        return physical, virtual


def resize_grid_metres(message, extent_m, expected_frame):
    """Keep every cell; convert virtual distances to a square physical BEV."""
    info = message.info
    if (message.header.frame_id != expected_frame or info.width <= 0
            or info.width != info.height or len(message.data) != info.width * info.height
            or not math.isfinite(info.resolution) or info.resolution <= 0.0
            or not math.isfinite(extent_m) or extent_m <= 0.0):
        raise ValueError('expected a valid square Vehicle-frame OccupancyGrid')
    origin = info.origin
    values = (origin.position.x, origin.position.y, origin.position.z,
              origin.orientation.x, origin.orientation.y, origin.orientation.z,
              origin.orientation.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite BEV origin')
    norm = sum(value * value for value in values[3:])
    if abs(norm - 1.0) > 1e-3:
        raise ValueError('BEV origin must have a unit quaternion')
    result = deepcopy(message)
    ratio = extent_m / (info.width * info.resolution)
    result.info.resolution = extent_m / info.width
    result.info.origin.position.x *= ratio
    result.info.origin.position.y *= ratio
    result.info.origin.position.z *= ratio
    return result


class ModelRealBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('model_real_bridge', **kwargs)
        defaults = {
            'motive_pose_topic': '/motive/vehicle/pose',
            'motive_yaw_is_orientation_z': True,
            'route_name': 'scenario3', 'map_scale': 0.1,
            'model_frame_id': 'model_map', 'odom_topic': '/model/vehicle/odom_map',
            'isaac_pose_topic': '/isaac/vehicle/pose',
            'isaac_grid_topic': '/bev/occupancy_grid',
            'model_grid_topic': '/model/bev/occupancy_grid',
            'bev_extent_m': 4.0, 'expected_grid_frame_id': 'Vehicle',
            'input_timeout_sec': 0.2, 'velocity_lpf_alpha': 0.4,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        p = self.settings
        topic_names = ('motive_pose_topic', 'odom_topic', 'isaac_pose_topic',
                       'isaac_grid_topic', 'model_grid_topic')
        if any(not p[name] for name in topic_names) or len({p[name] for name in topic_names}) != 5:
            raise ValueError('bridge input and output topics must be nonempty and distinct')
        if (not math.isfinite(p['input_timeout_sec']) or p['input_timeout_sec'] <= 0
                or not math.isfinite(p['bev_extent_m']) or p['bev_extent_m'] <= 0
                or not math.isfinite(p['velocity_lpf_alpha'])
                or not 0.0 <= p['velocity_lpf_alpha'] <= 1.0):
            raise ValueError('invalid timeout, BEV extent or velocity filter')
        directory = Path(get_package_share_directory('virtual_control')) / 'data/knu_routes'
        starts = load_start_positions(directory)
        rows = json.loads((directory / 'knu_start_poses.json').read_text())['starts']
        row = next(row for row in rows if row['scenario'] == p['route_name'])
        self.alignment = ModelAlignment(starts[p['route_name']], row['ros_heading_rad'],
                                        p['map_scale'])
        self.previous = None
        self.filtered_speed = None
        self.last_pose_time = None
        self.odom_pub = self.create_publisher(Odometry, p['odom_topic'], 10)
        self.isaac_pub = self.create_publisher(PoseStamped, p['isaac_pose_topic'], 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, p['model_grid_topic'], 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, p['motive_pose_topic'], self.pose_callback, qos_profile_sensor_data)
        self.grid_sub = self.create_subscription(
            OccupancyGrid, p['isaac_grid_topic'], self.grid_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f"Model bridge: {p['motive_pose_topic']} -> {p['odom_topic']}; "
            f"Isaac pose={p['isaac_pose_topic']}, physical BEV={p['bev_extent_m']}m square")

    def pose_callback(self, message):
        p = self.settings
        position, q = message.pose.position, message.pose.orientation
        if p['motive_yaw_is_orientation_z']:
            yaw = q.z
        else:
            values = (q.x, q.y, q.z, q.w)
            norm = sum(value * value for value in values)
            if not all(math.isfinite(value) for value in values) or abs(norm - 1.0) > 1e-3:
                return
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        received = time.monotonic()
        # Drop reordered stamped poses before they can move the virtual camera.
        if self.previous and stamp_ns > 0 and self.previous[2] > 0 and stamp_ns <= self.previous[2]:
            return
        try:
            physical, virtual = self.alignment.transform(position.x, position.y, position.z, yaw)
        except ValueError:
            return
        stamp = self.get_clock().now().to_msg()
        isaac = PoseStamped()
        isaac.header.stamp = stamp
        isaac.header.frame_id = 'map'
        isaac.pose.position.x, isaac.pose.position.y, isaac.pose.position.z = virtual[:3]
        isaac.pose.orientation.z = virtual[3]  # Existing Isaac yaw-scalar convention.
        self.isaac_pub.publish(isaac)
        self.last_pose_time = received
        previous = self.previous
        self.previous = (physical, received, stamp_ns)
        if previous is None:
            return  # At least two samples are required for measured speed.
        dt = ((stamp_ns - previous[2]) / 1e9 if stamp_ns > 0 and previous[2] > 0
              else received - previous[1])
        if not 1e-4 < dt <= p['input_timeout_sec']:
            self.filtered_speed = None
            return
        measured = math.hypot(physical[0] - previous[0][0], physical[1] - previous[0][1]) / dt
        alpha = p['velocity_lpf_alpha']
        self.filtered_speed = (measured if self.filtered_speed is None else
                               alpha * measured + (1 - alpha) * self.filtered_speed)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = p['model_frame_id']
        odom.child_frame_id = 'model_vehicle'
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = physical[:3]
        odom.pose.pose.orientation.z = physical[3]  # Controllers explicitly expect scalar yaw.
        odom.twist.twist.linear.x = self.filtered_speed
        odom.twist.twist.angular.z = wrap(physical[3] - previous[0][3]) / dt
        self.odom_pub.publish(odom)

    def grid_callback(self, message):
        p = self.settings
        if self.last_pose_time is None or time.monotonic() - self.last_pose_time > p['input_timeout_sec']:
            return
        try:
            output = resize_grid_metres(message, p['bev_extent_m'], p['expected_grid_frame_id'])
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.grid_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ModelRealBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
