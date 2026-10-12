"""Isolated fake-pose tests. Opt in with KNU_BATCH_ROS_TEST=1, ROS_DOMAIN_ID=177."""
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('KNU_BATCH_ROS_TEST') != '1',
                                reason='opt-in isolated ROS integration')
PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))


@pytest.fixture
def harness(tmp_path, monkeypatch):
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    import knu_batch_runner as module
    clock = [100.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    rclpy.init()
    fake = Node('px4_odom_map_bridge')
    fake.declare_parameter('route_name', 'scenario3')
    runner = module.KnuBatchRunner(parameter_overrides=[
        Parameter('routes', value=['scenario3']), Parameter('repeat_count', value=10),
        Parameter('require_recorder', value=True),
        Parameter('results_directory', value=str(tmp_path))])
    runner.timer.cancel()  # Deterministic accelerated time; DDS is still real.
    alignments = []

    def changed(parameters):
        assert parameters[0].value == 'scenario3'
        assert runner.pose[2] < 0.2
        alignments.append(runner.token)
        return SetParametersResult(successful=True)

    fake.add_on_set_parameters_callback(changed)
    odom_pub = fake.create_publisher(Odometry, '/motive/vehicle/odom_map', 10)
    grid_pub = fake.create_publisher(OccupancyGrid, '/bev/occupancy_grid', 10)
    ref_pub = fake.create_publisher(PathMsg, '/planner/lower_reference_path', runner.qos)
    permissions = []
    fake.create_subscription(Bool, '/knu_batch/run_permission',
                             lambda msg: permissions.append(msg.data), 10)
    fake.create_subscription(PathMsg, '/navigation/global_path', lambda msg: None, runner.qos)

    def step(position=None, speed=0.0, recorder=True, dt=0.25):
        clock[0] += dt
        msg = Odometry(); msg.header.frame_id = 'map'
        x, y = position or runner.start_positions['scenario3'][:2]
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.twist.twist.linear.x = float(speed)
        grid = OccupancyGrid(); grid.info.width = grid.info.height = 2
        grid.info.resolution = 1.0; grid.data = [0]*4
        reference = PathMsg(); reference.header.frame_id = 'map'
        reference.poses = [PoseStamped(), PoseStamped()]
        odom_pub.publish(msg); grid_pub.publish(grid); ref_pub.publish(reference)
        # Callbacks use the same data; actual DDS discovery and parameter calls remain live.
        runner.on_pose(msg); runner.on_grid(grid); runner.on_reference(reference)
        if recorder is not None:
            runner.on_recorder(Bool(data=recorder))
        for _ in range(2):
            rclpy.spin_once(fake, timeout_sec=0.002)
            rclpy.spin_once(runner, timeout_sec=0.002)
        runner.tick()

    value = SimpleNamespace(runner=runner, fake=fake, step=step, clock=clock,
                            alignments=alignments, permissions=permissions)
    try:
        yield value
    finally:
        runner.destroy_node(); fake.destroy_node(); rclpy.shutdown()


def reach_running(h):
    for _ in range(300):
        h.step()
        if h.runner.phase == 'running':
            return
    pytest.fail(f'never started: {h.runner.phase}')


def test_ten_identical_routes_wait_for_coasting_and_final_stop(harness):
    h = harness; runner = h.runner
    for attempt in range(10):
        reach_running(h)
        assert runner.index == attempt
        points = runner.points['scenario3']
        for a, b in zip(points, points[1:]):
            steps = math.ceil(math.dist(a[:2], b[:2])/1.0)
            for j in range(steps):
                p = [a[i]+(b[i]-a[i])*j/steps for i in range(2)]
                h.step(p, 4.0)
                if runner.phase != 'running':
                    break
            if runner.phase != 'running':
                break
        assert runner.phase == 'coasting'
        assert runner.rows[-1]['outcome'] == 'completed'
        token = runner.token
        for _ in range(88):  # 22 s rolling: the old 10 s stop timeout was insufficient.
            h.step(points[-1][:2], 1.0)
            assert runner.phase == 'coasting' and runner.token == token
        for _ in range(6):
            h.step(points[-1][:2], 0.0)
        assert runner.rows[-1]['stop_status'] == 'stationary'
    assert runner.phase == 'done'
    assert len(runner.rows) == len(h.alignments) == len(set(h.alignments)) == 10
    assert all(row['route'] == 'scenario3' for row in runner.rows)
    assert len((runner.output/'summary.csv').read_text().splitlines()) == 11


def test_recorder_missing_never_allows_start_and_loss_aborts(harness):
    h = harness
    for _ in range(20):
        h.step(recorder=False)
    assert h.runner.phase == 'startup' and not h.alignments
    reach_running(h)
    for _ in range(10):
        h.step(speed=1.0, recorder=None)
    assert h.runner.phase == 'aborting'
    assert h.runner.rows[-1]['outcome'] == 'batch_aborted'
    for _ in range(6):
        h.step(recorder=False)
    assert h.runner.phase == 'aborted' and len(h.alignments) == 1
    assert not h.permissions[-1]


def test_stop_timeout_never_reanchors_next_attempt(harness):
    h = harness
    reach_running(h)
    h.runner.fail_or_finish('off_route')
    for _ in range(245):
        h.step(speed=2.0)
    assert h.runner.phase == 'aborted'
    assert len(h.alignments) == 1 and len(h.runner.rows) == 1
    assert h.runner.rows[-1]['stop_status'] == 'timeout'


def test_real_recorder_discovery_chunk_rotation_and_exact_grid_messages(tmp_path):
    import signal
    import subprocess
    import time
    import rclpy
    import rosbag2_py
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from imac_interfaces.msg import PathWithArcLength
    from std_msgs.msg import Float64MultiArray, String
    from drive_debug_profile import REQUIRED_TOPICS, recorder_command
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    rclpy.init()
    node = Node('essential_recorder_test')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    kinds = (OccupancyGrid, Odometry, PathMsg, PathWithArcLength,
             Float64MultiArray, String, String)
    pubs = {topic: node.create_publisher(kind, topic, qos if kind is PathMsg else 10)
            for topic, kind in zip(REQUIRED_TOPICS, kinds)}
    unlisted = node.create_publisher(String, '/camera/not_for_recording', 10)
    path = PathMsg(); path.header.frame_id = 'map'
    pubs['/navigation/global_path'].publish(path)  # Only once, before recorder discovery.
    command = recorder_command(tmp_path/'bag')
    command[command.index('--max-bag-duration')+1] = '1'
    stream = (tmp_path/'recorder.log').open('w')
    process = subprocess.Popen(command, stdout=stream, stderr=stream)
    expected = []
    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if all(any(s.node_name == 'rosbag2_recorder' for s in
                       node.get_subscriptions_info_by_topic(topic)) for topic in REQUIRED_TOPICS):
                break
        else:
            pytest.fail('recorder did not discover required topics')
        for i in range(90):
            msg = OccupancyGrid(); msg.header.stamp.sec = i+1
            msg.info.width = msg.info.height = 128; msg.info.resolution = 0.3125
            msg.data = [0]*16384; msg.data[i] = 100
            expected.append(msg)
            pubs['/bev/occupancy_grid'].publish(msg)
            unlisted.publish(String(data='must not be recorded'))
            rclpy.spin_once(node, timeout_sec=0.04)
        deadline = time.monotonic()+0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        stream.close(); node.destroy_node(); rclpy.shutdown()
    assert process.returncode == 0
    assert len(list((tmp_path/'bag').glob('*.mcap'))) >= 2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    actual = []; paths = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        assert topic != '/camera/not_for_recording'
        if topic == '/bev/occupancy_grid':
            # CDR padding may differ across separate serializations; compare all fields.
            actual.append(deserialize_message(data, OccupancyGrid))
        if topic == '/navigation/global_path':
            paths += 1
    assert actual == expected
    assert paths >= 1
