#!/usr/bin/env python3
"""Publish the KNU waypoint missions without a MATLAB runtime."""

import csv
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_route(route_name, routes_directory):
    """Load one unmodified map-frame mission; reject incomplete route files."""
    if route_name not in tuple(f'scenario{i}' for i in range(1, 22)):
        raise ValueError('route_name must be scenario1 through scenario21')
    filename = Path(routes_directory) / (route_name + '.csv')
    points = []
    with filename.open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ['x', 'y', 'z']:
            raise ValueError(f'{filename}: expected CSV header x,y,z')
        for line_number, row in enumerate(reader, 2):
            try:
                point = tuple(float(row[axis]) for axis in ('x', 'y', 'z'))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'{filename}:{line_number}: invalid waypoint') from error
            if None in row or not all(math.isfinite(value) for value in point):
                raise ValueError(f'{filename}:{line_number}: invalid waypoint')
            if points and point[:2] == points[-1][:2]:
                raise ValueError(f'{filename}:{line_number}: consecutive duplicate waypoint')
            points.append(point)
    if len(points) < 2:
        raise ValueError(f'{filename}: a route needs at least two waypoints')
    return points


def load_start_positions(routes_directory):
    """Vehicle spawn positions, independent of the unmodified waypoint CSVs."""
    filename = Path(routes_directory) / 'knu_start_poses.json'
    data = json.loads(filename.read_text(encoding='utf-8'))
    expected = {f'scenario{i}' for i in range(1, 22)}
    positions = {}
    for row in data['starts']:
        name = row['scenario']
        point = tuple(float(x) for x in row['position'])
        if (name not in expected or name in positions or len(point) != 3
                or not all(math.isfinite(x) for x in point)):
            raise ValueError(f'{filename}: invalid or duplicate start position')
        positions[name] = point
    if set(positions) != expected:
        raise ValueError(f'{filename}: expected exactly scenario1 through scenario21')
    return positions


class KnuGlobalPathPublisher(Node):
    """Keep the selected mission available to late or restarted subscribers."""

    def __init__(self, **kwargs):
        super().__init__('knu_global_path_publisher', **kwargs)
        self.declare_parameter('route_name', 'scenario1')
        self.declare_parameter('global_path_topic', '/navigation/global_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_sec', 2.0)

        route_name = self.get_parameter('route_name').value
        topic = self.get_parameter('global_path_topic').value
        frame_id = self.get_parameter('frame_id').value
        period = self.get_parameter('publish_period_sec').value
        if not frame_id or not math.isfinite(period) or period < 0.0:
            raise ValueError('frame_id must be nonempty and publish_period_sec finite >= 0')

        route_directory = (
            Path(get_package_share_directory('virtual_control')) / 'data' / 'knu_routes'
        )
        points = load_route(route_name, route_directory)
        self.path = PathMsg()
        self.path.header.frame_id = frame_id
        for x, y, z in points:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            # Match MATLAB makePathMsg: positions define the route, quaternion is identity.
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(PathMsg, topic, qos)
        self.publish_path()
        # A heartbeat also serves volatile visualization subscribers. Zero is latch-only.
        self.timer = self.create_timer(period, self.publish_path) if period > 0.0 else None
        self.get_logger().info(
            f'KNU {route_name}: {len(points)} waypoints -> {topic}, frame={frame_id}'
        )

    def publish_path(self):
        """Refresh timestamps without changing the mission coordinates or order."""
        stamp = self.get_clock().now().to_msg()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp
        self.path_publisher.publish(self.path)


def main(args=None):
    """Run the independent global path publisher."""
    rclpy.init(args=args)
    node = None
    try:
        node = KnuGlobalPathPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
