"""Opt-in synthetic-grid tests of the actual main9 upper-planner executable."""

import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('INTERVAL_CENTERING_ROS_TEST') != '1',
    reason='opt-in isolated ROS integration')


def test_main9_observation_state_and_interval_processing(tmp_path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Float64MultiArray

    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '1')
    executable = Path(__file__).resolve().parents[3] / 'build/virtual_control/upper_planner_node'
    assert executable.is_file()
    rclpy.init()
    node = rclpy.create_node('main9_interval_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(PathMsg, '/main9_test/path', qos)
    grid_pub = node.create_publisher(OccupancyGrid, '/main9_test/grid', 10)
    odom_pub = node.create_publisher(Odometry, '/main9_test/odom', 10)
    states, details, legacy = [], [], []
    state_columns = []

    def receive_state(msg):
        state_columns[:] = msg.layout.dim[0].label.split(',')
        states.append(list(msg.data))

    node.create_subscription(Float64MultiArray, '/debug/upper_interval_centering_state',
                             receive_state, 10)
    node.create_subscription(Float64MultiArray, '/debug/upper_interval_info',
                             lambda msg: details.append(list(msg.data)), 10)
    node.create_subscription(Float64MultiArray, '/debug/upper_constraint_intervals',
                             lambda msg: legacy.append(list(msg.data)), 10)
    path = PathMsg()
    path.header.frame_id = 'map'
    for x in (0.0, 40.0, 80.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.pose.pose.orientation.w = 1.0
    odom.twist.twist.linear.x = 1.5
    current_grid = None
    process = None
    log_stream = None
    logfile = None

    def grid_for(specs):
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_link'
        grid.info.resolution = 0.1
        grid.info.width = 180
        grid.info.height = 200
        grid.info.origin.position.x = -1.0
        grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        data = []
        for gy in range(200):
            y = -10.0 + (gy + 0.5) * 0.1
            for gx in range(180):
                x = -1.0 + (gx + 0.5) * 0.1
                k = min(5, max(0, math.floor((x + 1.5) / 3.0)))
                spec = specs[k]
                if spec is None:
                    value = -1
                elif spec == 'unknown_boundary':
                    value = 100 if abs(y) < 2.0 else -1
                elif isinstance(spec, list):
                    value = 100 if any(lo < y < hi for lo, hi in spec) else 0
                else:
                    value = 100 if abs(y) < spec / 2 else 0
                data.append(value)
        grid.data = data
        return grid

    def stop():
        nonlocal process, log_stream
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            process = None
        if log_stream is not None:
            log_stream.close()
            log_stream = None

    def start(name, specs):
        nonlocal process, log_stream, logfile, current_grid
        stop()
        until = time.monotonic() + 0.3
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.02)
        states.clear()
        details.clear()
        legacy.clear()
        current_grid = grid_for(specs)
        params = {
            'scale_mode': 'fullscale', 'upper_preview_steps': 5,
            'upper_prediction_dt_sec': 2.0, 'target_speed_mps': 1.5,
            'upper_planner_rate_hz': 5.0, 'preview_limit_to_grid_extent': False,
            'preview_restore_full_horizon': True, 'input_timeout_sec': 0.5,
            'global_path_topic': '/main9_test/path', 'grid_map_topic': '/main9_test/grid',
            'odom_topic': '/main9_test/odom', 'preview_interval_debug': True,
        }
        args = [str(executable), '--ros-args']
        for key, value in params.items():
            text = str(value).lower() if isinstance(value, bool) else str(value)
            args += ['-p', f'{key}:={text}']
        logfile = tmp_path / f'{name}.log'
        log_stream = logfile.open('w')
        process = subprocess.Popen(args, stdout=log_stream, stderr=subprocess.STDOUT)

    def wait_for(predicate, timeout=12, publish_grid=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, logfile.read_text()
            stamp = node.get_clock().now().to_msg()
            path.header.stamp = odom.header.stamp = stamp
            path_pub.publish(path)
            odom_pub.publish(odom)
            if publish_grid:
                current_grid.header.stamp = stamp
                grid_pub.publish(current_grid)
            rclpy.spin_once(node, timeout_sec=0.03)
            if predicate():
                return
        pytest.fail(f'Condition timed out. States: {states[-8:]}\n{logfile.read_text()}')

    def rows():
        return [details[-1][i:i+12] for i in range(0, len(details[-1]), 12)]

    try:
        start('far_narrow_road', [10, 10, 10, 10, 10, 4.9])
        wait_for(lambda: states and details and states[-1][2] > 0)
        assert state_columns == ['stamp', 'B_before', 'B', 'threshold', 'candidate',
                                 'candidate_step', 'candidate_interval']
        assert len(states[-1]) == 7
        baseline = states[-1][2]
        assert baseline == pytest.approx(4.8, abs=0.11)
        assert states[-1][3] == pytest.approx(1.5 * baseline)
        assert states[-1][5:7] == [5.0, 0.0]
        assert all(bool(row[4]) == (int(row[0]) == 5) for row in rows())
        assert all(row[5:7] == [1.0, 1.0] for row in rows())
        for row in rows():
            assert row[3] == pytest.approx(row[2] * 0.4 if row[4] else row[2] - 4.0)

        # Wide-only observations cannot grow B past the previous 1.5*B limit.
        current_grid = grid_for([10]*6)
        start_index = len(states)
        wait_for(lambda: len([s for s in states[start_index:] if s[4] > 9]) >= 3)
        assert all(s[2] == pytest.approx(baseline)
                   for s in states[start_index:] if s[4] > 9)

        # A shorter non-reference branch does not win or freeze B. Extract the
        # full preview even beyond a blocked stage; preserve the solver prefix.
        current_grid = grid_for([8, 8, 8, 0, 8, [(-2, 2), (5, 6)]])
        start_index = len(states)
        wait_for(lambda: any(s[5:7] == [5.0, 0.0] and s[4] < 4.1
                             for s in states[start_index:]) and
                 details and any(r[0] == 5 and r[1] == 1 for r in rows()) and
                 legacy and max(legacy[-1][0::7]) == 2)
        narrowed = next(s for s in states[start_index:] if s[4] < 4.1)
        assert narrowed[2] == pytest.approx(3.9, abs=0.11)
        assert narrowed[2] == narrowed[4]
        assert all(r[0] != 3 for r in rows())  # Blocked reference gets no fallback.
        branch = next(r for r in rows() if r[0] == 5 and r[1] == 1)
        assert branch[2] < narrowed[4] and branch[9] == 0

        current_grid = grid_for([None]*6)
        start_index = len(states)
        wait_for(lambda: any(math.isnan(s[4]) for s in states[start_index:]) and
                 details and all(r[8] == 1 for r in rows()))
        missing = next(s for s in states[start_index:] if math.isnan(s[4]))
        assert missing[2] == pytest.approx(narrowed[2])
        assert missing[5:7] == [-1.0, -1.0]
        assert all(r[4] == 1 and r[11] == 9.0 for r in rows())
        assert all(r[2] == pytest.approx(4.5) for r in rows())

        # A stale required grid retains B; resumed observations above its limit
        # must not reinitialize it.
        deadline = time.monotonic() + 0.8
        wait_for(lambda: time.monotonic() >= deadline, publish_grid=False)
        current_grid = grid_for([10]*6)
        start_index = len(states)
        wait_for(lambda: any(s[4] > 9 for s in states[start_index:]))
        assert next(s for s in states[start_index:] if s[4] > 9)[2] == pytest.approx(narrowed[2])

        # Without B, clipped and unknown-boundary observations stay OFF.
        for name, specs in (
            ('clipped', [30]*6),
            ('unknown_boundary', ['unknown_boundary']*6),
            ('reference_outside', [[(1, 2)]]*6),
        ):
            start(name, specs)
            wait_for(lambda: states and details)
            assert states[-1][2] == 0
            assert all(r[4] == 0 and math.isnan(r[11]) for r in rows())
            for row in rows():
                assert row[3] == pytest.approx(row[2] - 2*min(2.0, 0.45*row[2]))
            if name != 'reference_outside':
                assert all(r[7] == 1 for r in rows())
            else:
                assert all(r[9] == 0 for r in rows())

        start('skip_blocked_k0', [0, 4, 4, 15, 4, 4])
        wait_for(lambda: states and details)
        assert states[0][5] == 1
        assert states[0][2] > 0
        assert all(r[0] > 0 and r[8] == 0 for r in rows())
    finally:
        stop()
        node.destroy_node()
        rclpy.shutdown()
