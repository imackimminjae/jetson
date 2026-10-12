"""Actual upper + runner, recorded start BEV, fake vehicle/bridge; no actuator node."""
import json
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
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))


def recorded_start_grid():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid
    if not (ROOT/'drive_batches/20261008_055617_scenario3_73cb2c/bag/metadata.yaml').exists():
        pytest.skip('recorded scenario3 regression fixture not available')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(ROOT/
        'drive_batches/20261008_055617_scenario3_73cb2c/bag'), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
    while reader.has_next():
        _, blob, stamp = reader.read_next()
        if stamp/1e9 >= 1791406585.9:
            return deserialize_message(blob, OccupancyGrid)
    raise AssertionError('recorded start grid missing')


def test_actual_upper_ten_repeated_missions_and_heartbeat(tmp_path):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.msg import SetParametersResult
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from std_msgs.msg import Bool, String, Float64MultiArray
    from std_srvs.srv import Trigger
    import yaml
    from knu_batch_runner import KnuBatchRunner
    from repeat_drive_session import effective_config

    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    grid = recorded_start_grid()
    rclpy.init()
    fake = Node('px4_odom_map_bridge')
    fake.declare_parameter('route_name', 'scenario3')
    runner = KnuBatchRunner(parameter_overrides=[
        Parameter('routes', value=['scenario3']), Parameter('repeat_count', value=10),
        Parameter('reset_upper_each_attempt', value=True),
        Parameter('results_directory', value=str(tmp_path/'results'))])
    upper_config = effective_config(10, tmp_path)['upper_planner_node']
    upper_config['ros__parameters']['upper_planner_rate_hz'] = 10.0
    config = tmp_path/'upper.yaml'
    config.write_text(yaml.safe_dump({'upper_planner_node': upper_config}))
    log = (tmp_path/'upper.log').open('w')
    executable = os.environ.get('BATCH_TEST_UPPER', str(ROOT/'build/virtual_control/upper_planner_node'))
    process = subprocess.Popen([executable, '--ros-args', '--params-file', str(config)],
                               stdout=log, stderr=log)
    odom_pub = fake.create_publisher(Odometry, '/motive/vehicle/odom_map', 10)
    grid_pub = fake.create_publisher(OccupancyGrid, '/bev/occupancy_grid', 10)
    permissions = []
    fake.create_subscription(Bool, '/knu_batch/run_permission',
                             lambda m: permissions.append(m.data), 10)
    events, widths, references, alignments = [], [], [], []
    fake.create_subscription(String, '/debug/upper_branch_event',
                             lambda m: events.append(json.loads(m.data)), 100)
    fake.create_subscription(Float64MultiArray, '/debug/upper_interval_centering_state',
                             lambda m: widths.append(list(m.data)), 100)
    fake.create_subscription(PathMsg, '/planner/lower_reference_path', references.append, runner.qos)
    reset_client = fake.create_client(Trigger, '/upper_planner_node/reset_mission')
    points = runner.points['scenario3']
    lengths = [math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:])]
    state = dict(distance=0.0, position=points[0][:2], yaw=-math.pi/2,
                 last_running=False)

    def changed(parameters):
        assert parameters[0].value == 'scenario3'
        assert runner.pose[2] < 0.2
        state.update(distance=0.0, position=points[0][:2], yaw=-math.pi/2)
        alignments.append(runner.token)
        return SetParametersResult(successful=True)

    fake.add_on_set_parameters_callback(changed)
    first_events = {}; heartbeat_checks = set(); rejected_moving_reset = False
    moving_reset = None; clock_previous = 0.0; deadline = time.monotonic()+160
    started_running = {}; end_pairs = {}; baselines = []; stale_checked = False
    try:
        while runner.phase not in ('done', 'aborted') and time.monotonic() < deadline:
            now = time.monotonic()
            assert process.poll() is None, (tmp_path/'upper.log').read_text()
            if now-clock_previous >= 0.05:
                clock_previous = now
                running = runner.phase == 'running'
                if running:
                    started_running.setdefault(runner.index, now)
                    # Exercise duplicate path heartbeats while the mission is in progress.
                    if runner.index not in heartbeat_checks and now-started_running[runner.index] > .3:
                        revision = events[-1]['path_revision']
                        runner.send_path()
                        heartbeat_checks.add(runner.index)
                        baselines.append((runner.index, revision))
                    state['distance'] += 3.0
                    distance = state['distance']
                    for a, b, length in zip(points, points[1:], lengths):
                        if distance <= length:
                            state['position'] = [a[i]+(b[i]-a[i])*distance/length for i in range(2)]
                            state['yaw'] = math.atan2(b[1]-a[1], b[0]-a[0])
                            break
                        distance -= length
                    else:
                        state['position'] = points[-1][:2]
                elif state['last_running']:
                    end_pairs[runner.index] = [events[-1]['wp0'], events[-1]['wp1']]
                state['last_running'] = running
                msg = Odometry(); msg.header.frame_id = 'map'
                msg.header.stamp = fake.get_clock().now().to_msg()
                msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, state['position'])
                msg.pose.pose.orientation.z = float(state['yaw'])
                msg.twist.twist.linear.x = 6.0 if running else 0.0
                odom_pub.publish(msg)
                grid.header.stamp = msg.header.stamp; grid_pub.publish(grid)
                if running and runner.index == 0 and now-started_running[0] > 1 and moving_reset is None:
                    moving_reset = reset_client.call_async(Trigger.Request())
            if moving_reset is not None and moving_reset.done():
                assert not moving_reset.result().success
                rejected_moving_reset = True
            if runner.phase == 'reference' and not stale_checked:
                stale = PathMsg(); stale.header.frame_id = 'map'
                from geometry_msgs.msg import PoseStamped
                stale.poses = [PoseStamped(), PoseStamped()]
                old = runner.reference_at
                runner.on_reference(stale)
                assert runner.reference_at == old
                stale_checked = True
            rclpy.spin_once(fake, timeout_sec=0.003)
            rclpy.spin_once(runner, timeout_sec=0.003)
            for e in events:
                first_events.setdefault(e['path_revision'], e)
        assert runner.phase == 'done', (runner.phase, runner.rows)
        assert len(runner.rows) == len(set(alignments)) == 10
        assert all(row['outcome'] == 'completed' for row in runner.rows)
        assert all(row['stop_status'] == 'stationary' for row in runner.rows)
        assert len(first_events) == 10  # Heartbeats never create extra missions.
        assert len(heartbeat_checks) == 10 and rejected_moving_reset and stale_checked
        assert all(pair == [6, 7] for pair in end_pairs.values())
        plans = []
        for revision, event in sorted(first_events.items()):
            assert event['valid_plan'] and event['wp0'] <= 1 and event['wp1'] <= 2
            assert event['x'] == pytest.approx(points[0][0], abs=1e-6)
            assert event['y'] == pytest.approx(points[0][1], abs=1e-6)
            state_row = min(widths, key=lambda row: abs(row[0]-event['t_emit']))
            assert state_row[1] == 0.0  # B starts empty on every new mission.
            plans.append(event['published_world'])
        for plan in plans[1:]:
            np.testing.assert_allclose(plan, plans[0], atol=1e-7, rtol=0)
        for index, revision in baselines:
            assert revision in first_events
        (tmp_path/'evidence.json').write_text(json.dumps({
            'first_events': first_events, 'end_pairs': end_pairs, 'rows': runner.rows,
            'moving_reset_rejected': rejected_moving_reset, 'old_reference_rejected': stale_checked,
            'heartbeat_missions': len(first_events)}, indent=2))
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=10)
        log.close(); runner.destroy_node(); fake.destroy_node(); rclpy.shutdown()
