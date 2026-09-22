"""Opt-in isolated ROS integration checks. Never connect to PX4 or vehicle outputs.

ROS_DOMAIN_ID=173 KNU_BATCH_ROS_TEST=1 python3 -m pytest -q test_knu_batch_ros.py
"""

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('KNU_BATCH_ROS_TEST') != '1',
                                reason='opt-in isolated ROS integration')
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))


def test_runner_continues_after_completion_stop_deviation_and_lost_pose(tmp_path):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from std_msgs.msg import Bool
    from knu_batch_runner import KnuBatchRunner

    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    rclpy.init()
    fake = Node('px4_odom_map_bridge')
    fake.declare_parameter('route_name', 'scenario1')
    selected = ['scenario1']

    def changed(parameters):
        selected[0] = parameters[0].value
        return SetParametersResult(successful=True)

    fake.add_on_set_parameters_callback(changed)
    runner = KnuBatchRunner(parameter_overrides=[
        Parameter('routes', value=[
            'scenario1', 'scenario2', 'scenario3', 'scenario4', 'scenario21']),
        Parameter('results_directory', value=str(tmp_path)),
        Parameter('pose_loss_sec', value=2.0),
    ])
    odom_pub = fake.create_publisher(Odometry, '/motive/vehicle/odom_map', 10)
    grid_pub = fake.create_publisher(OccupancyGrid, '/bev/occupancy_grid', 10)
    ref_pub = fake.create_publisher(PathMsg, '/planner/lower_reference_path', runner.qos)
    permissions = []
    fake.create_subscription(Bool, '/knu_batch/run_permission',
                             lambda msg: permissions.append(msg.data), 10)
    have_path = [False]
    fake.create_subscription(PathMsg, '/navigation/global_path',
                             lambda msg: have_path.__setitem__(0, True), runner.qos)
    grid = OccupancyGrid()
    grid.header.frame_id = 'Vehicle'
    grid.info.width = grid.info.height = 2
    grid.info.resolution = 1.0
    grid.data = [0]*4
    reference = PathMsg()
    reference.header.frame_id = 'map'
    reference.poses = [PoseStamped(), PoseStamped()]
    last_route = None
    walked = lateral = 0.0
    next_publish = 0.0
    deadline = time.monotonic()+75
    try:
        while runner.phase != 'done' and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                next_publish = now+0.05
                route = runner.route or 'scenario1'
                if route != last_route:
                    walked = lateral = 0.0
                    last_route = route
                    have_path[0] = False
                points = runner.points.get(route, runner.points['scenario1'])
                if runner.phase == 'running':
                    if route in ('scenario1', 'scenario21'):
                        walked += 3.0
                    elif route == 'scenario3':
                        lateral = min(35.0, lateral+2.0)
                distance = walked
                pos = points[0][:2]
                for a, b in zip(points, points[1:]):
                    length = math.dist(a[:2], b[:2])
                    if distance <= length:
                        pos = tuple(a[i]+(b[i]-a[i])*distance/length for i in range(2))
                        break
                    distance -= length
                if runner.phase != 'running':
                    pos = runner.start_positions[route][:2]
                msg = Odometry()
                msg.header.frame_id = 'map'
                msg.header.stamp = fake.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, pos)
                if runner.phase == 'running' and route == 'scenario3':
                    # First scenario3 segment heads south; leave perpendicular to it.
                    msg.pose.pose.position.x += lateral
                if not (runner.phase == 'running' and route == 'scenario4'):
                    odom_pub.publish(msg)
                grid.header.stamp = fake.get_clock().now().to_msg()
                grid_pub.publish(grid)
                if have_path[0]:
                    ref_pub.publish(reference)
            rclpy.spin_once(fake, timeout_sec=0.002)
            rclpy.spin_once(runner, timeout_sec=0.002)
        assert runner.phase == 'done'
        assert [row['outcome'] for row in runner.rows] == [
            'completed', 'stopped_10s', 'off_route', 'pose_timeout', 'completed']
        assert len((runner.output/'summary.csv').read_text().splitlines()) == 6
        assert True in permissions and False in permissions
        assert selected[0] == 'scenario21'
    finally:
        runner.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()


def test_real_bridge_reanchors_and_lower_permission_expires(tmp_path):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParametersAtomically
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float64MultiArray

    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    rclpy.init()
    node = Node('batch_transport_test')
    exe = Path(get_package_prefix('virtual_control'))/'lib/virtual_control'
    processes = []
    streams = []
    try:
        for executable, params in [
                ('px4_odom_map_bridge', ['enable_batch_reanchor:=true']),
                ('lower_tracking_mpc_node', ['batch_supervision:=true', 'mavlink_enable:=false',
                                             'pixhawk_output_backend:=disabled',
                                             'publish_pwm:=false'])]:
            command = [str(exe/executable), '--ros-args']
            for param in params:
                command.extend(['-p', param])
            stream = (tmp_path/(executable+'.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, stdout=stream, stderr=stream))
        raw = node.create_publisher(PoseStamped, '/px4/sih/odom', 10)
        run = node.create_publisher(Bool, '/knu_batch/run_permission', 10)
        poses, modes = [], []
        node.create_subscription(Odometry, '/px4/sih/odom_map', poses.append, 10)
        node.create_subscription(Float64MultiArray, '/debug/lower_mpc_trace',
                                 lambda msg: modes.append(int(msg.data[12])), 10)
        client = node.create_client(
            SetParametersAtomically, '/px4_odom_map_bridge/set_parameters_atomically')

        def spin_for(duration, permission=None):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                msg = PoseStamped()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.position.x, msg.pose.position.y = 12.0, -4.0
                raw.publish(msg)
                if permission is not None:
                    run.publish(Bool(data=permission))
                rclpy.spin_once(node, timeout_sec=0.03)

        spin_for(2)
        assert modes and modes[-1] == 4
        assert client.service_is_ready()
        from knu_global_path_publisher import load_start_positions
        starts = load_start_positions(PACKAGE/'data/knu_routes')
        for number in range(1, 22):
            name = f'scenario{number}'
            poses.clear()
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter('route_name', value=name).to_parameter_msg()]
            future = client.call_async(request)
            spin_for(0.4)
            assert future.done() and future.result().result.successful
            assert poses
            assert poses[-1].pose.pose.position.x == pytest.approx(starts[name][0])
            assert poses[-1].pose.pose.position.y == pytest.approx(starts[name][1])
        modes.clear()
        spin_for(0.5, True)
        assert any(mode != 4 for mode in modes)
        spin_for(1.5)
        assert modes[-1] == 4  # Heartbeat loss must stop without a false message.
        assert all(process.poll() is None for process in processes)
    finally:
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        for stream in streams:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
