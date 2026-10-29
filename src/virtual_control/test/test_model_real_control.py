"""Physical units and isolated synthetic Motive/Isaac integration; no hardware."""
from copy import deepcopy
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
import time

import pytest
import yaml
from nav_msgs.msg import OccupancyGrid

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from model_real_bridge import ModelAlignment, resize_grid_metres


def grid(extent=40.0):
    message = OccupancyGrid()
    message.header.frame_id = 'Vehicle'
    message.info.width = message.info.height = 128
    message.info.resolution = extent / 128
    message.info.origin.position.y = -extent / 2
    message.info.origin.orientation.w = 1.0
    message.data = [0] * (128 * 128)
    message.data[100] = 100
    message.data[101] = -1
    return message


def test_motive_metres_and_isaac_motion_use_consistent_scale():
    alignment = ModelAlignment((187.0, 220.0, 0.0), -math.pi / 2, 0.1)
    first, virtual_first = alignment.transform(1.0, 2.0, 0.1, 0.3)
    assert first[:3] == pytest.approx((18.7, 22.0, 0.0))
    assert virtual_first[:3] == pytest.approx((187.0, 220.0, 0.0))
    physical, virtual = alignment.transform(
        1.0 + 0.25 * math.cos(0.3), 2.0 + 0.25 * math.sin(0.3), 0.1, 0.3)
    assert physical[:2] == pytest.approx((18.7, 21.75))
    assert virtual[:2] == pytest.approx((187.0, 217.5))
    assert physical[3] == virtual[3] == pytest.approx(-math.pi / 2)


@pytest.mark.parametrize('extent', [40.0, 4.0])
def test_bev_has_four_metre_extent_without_resampling_or_double_scaling(extent):
    original = grid(extent)
    before = deepcopy(original)
    result = resize_grid_metres(original, 4.0, 'Vehicle')
    assert result.info.resolution == pytest.approx(0.03125)
    assert result.info.width * result.info.resolution == pytest.approx(4.0)
    assert result.info.origin.position.y == pytest.approx(-2.0)
    assert result.info.origin.orientation == original.info.origin.orientation
    assert list(result.data) == list(original.data)
    assert result.header == original.header
    assert original == before


@pytest.mark.parametrize('mutation', ['frame', 'rectangle', 'resolution', 'data', 'rotation'])
def test_bad_grid_is_rejected(mutation):
    message = grid()
    if mutation == 'frame':
        message.header.frame_id = 'map'
    elif mutation == 'rectangle':
        message.info.height = 64
    elif mutation == 'resolution':
        message.info.resolution = float('nan')
    elif mutation == 'data':
        message.data = [0]
    else:
        message.info.origin.orientation.w = 0.0
    with pytest.raises(ValueError):
        resize_grid_metres(message, 4.0, 'Vehicle')


@pytest.mark.parametrize('scale', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_map_scale_is_rejected(scale):
    with pytest.raises(ValueError):
        ModelAlignment((187, 220, 0), 0.0, scale)


@pytest.mark.parametrize('centred_bev', [False, True])
def test_physical_profile_with_real_controllers(tmp_path, centred_bev):
    if os.environ.get('MODEL_REAL_ROS_TEST') != '1':
        pytest.skip('isolated ROS opt-in required')
    assert os.environ.get('ROS_DOMAIN_ID') == '178'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path as PathMessage
    from std_msgs.msg import Float64MultiArray, String
    from imac_interfaces.msg import PathWithArcLength

    settings = yaml.safe_load((PACKAGE / 'config/model_real_control.yaml').read_text())
    upper = settings['upper_planner_node']['ros__parameters']
    target_speed = settings['lower_tracking_mpc_node']['ros__parameters']['target_speed_mps']
    # Exercise the measured physical speed above the planner's nominal 0.5 m/s:
    # fixed duty must remain 0.091 rather than being cancelled by the old PI.
    measured_speed = .65
    rclpy.init()
    probe = process = log = None
    references, traces = [], []
    duty_commands = []
    branch_events, b_samples = [], []
    try:
        probe = rclpy.create_node('model_real_test_probe')
        for topic, msgtype, received in (
            ('/planner/lower_reference_path/with_arclength', PathWithArcLength, references),
            ('/debug/lower_mpc_trace', Float64MultiArray, traces),
            ('/model/control/duty_cmd', Float64MultiArray, duty_commands),
            ('/debug/upper_branch_event', String, branch_events),
            ('/debug/upper_interval_b_samples', Float64MultiArray, b_samples),
        ):
            probe.create_subscription(msgtype, topic, received.append, 10)
        log = (tmp_path / 'control_launch.log').open('w')
        process = subprocess.Popen(
            ['ros2', 'launch', 'virtual_control', 'model_real_control.launch.py',
             'hardware_output:=false', f'record_root:={tmp_path / "recordings"}'],
            stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        pose_input = upper['state_input_type'] == 'pose_stamped'
        state_type = PoseStamped if pose_input else Odometry
        state_topic = upper['pose_stamped_topic'] if pose_input else upper['odom_topic']
        odom_pub = probe.create_publisher(state_type, state_topic, 10)
        grid_pub = probe.create_publisher(OccupancyGrid, upper['grid_map_topic'], 10)
        path_pub = probe.create_publisher(
            PathMessage, upper['global_path_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        external_path = PathMessage()
        external_path.header.frame_id = upper['global_path_frame_alias']
        # Feed a straight +x test route. The physical profile now consumes the
        # external world points unchanged; custom profiles may still use a bias.
        correction_enabled = upper.get('enable_waypoint_bias', False)
        angle = upper.get('waypoint_yaw_bias_rad', 0.) if correction_enabled else 0.
        tx = upper.get('waypoint_x_bias_m', 0.) if correction_enabled else 0.
        ty = upper.get('waypoint_y_bias_m', 0.) if correction_enabled else 0.
        for x in (1.0, 31.0):
            point = PoseStamped()
            point.header.frame_id = external_path.header.frame_id
            point.pose.position.x = 1.0 - tx + math.cos(angle) * (x - 1.0)
            point.pose.position.y = 2.0 - ty - math.sin(angle) * (x - 1.0)
            point.pose.orientation.w = 1.0
            external_path.poses.append(point)
        start = time.monotonic()
        while time.monotonic() - start < 10.0:
            assert process.poll() is None, 'control launch exited; inspect log'
            elapsed = time.monotonic() - start
            odom = state_type()
            odom.header.stamp = probe.get_clock().now().to_msg()
            odom.header.frame_id = upper['path_frame_id']
            state_pose = odom.pose if pose_input else odom.pose.pose
            state_pose.position.x = 1.0 + measured_speed * elapsed
            state_pose.position.y = 2.0
            state_pose.orientation.w = 0.0
            if not pose_input:
                odom.twist.twist.linear.x = measured_speed
            odom_pub.publish(odom)
            extent = upper['bev_forward_m']
            image = grid(extent)
            if centred_bev:
                image.info.origin.position.x = -extent / 2
            image.data = [0] * len(image.data)
            image.header.stamp = odom.header.stamp
            grid_pub.publish(image)
            external_path.header.stamp = odom.header.stamp
            path_pub.publish(external_path)
            rclpy.spin_once(probe, timeout_sec=0.002)
            time.sleep(0.015)
        assert references and any(len(msg.path.poses) >= 2 for msg in references)
        assert traces and any(msg.data[10] == 1.0 for msg in traces), 'no valid lower MPC solve'
        events = [json.loads(msg.data) for msg in branch_events]
        physical_plans = [event for event in events
                          if event['valid_plan'] and event['vehicle_rollout_valid']]
        assert physical_plans, 'no published physical-rollout plan with real lower feedback'
        assert any(event['horizon'] == upper['upper_preview_steps']
                   for event in physical_plans), 'full preview did not fit inside physical BEV'
        assert all(event['vehicle_rollout_enabled']
                   and event['k1_extra_centering_disabled'] for event in physical_plans)
        rate_limit = settings['upper_planner_node']['ros__parameters'][
            'upper_max_steering_rate_radps']
        assert all(event['vehicle_max_command_rate_radps'] <= rate_limit + 1e-6
                   and event['vehicle_max_effective_rate_radps'] <= rate_limit + 1e-6
                   for event in physical_plans)
        assert any(len(msg.data) // 14 > 8 for msg in b_samples), 'no extra B probes'
        valid_traces = [msg for msg in traces if msg.data[10] == 1.0]
        expected_speed = measured_speed
        # Allow the position-derived velocity LPF to settle after startup.
        settled_traces = [msg for msg in valid_traces
                          if msg.data[0] - valid_traces[0].data[0] >= 1.0]
        assert settled_traces
        assert all(msg.data[4] == pytest.approx(expected_speed, abs=0.03)
                   for msg in settled_traces)
        assert all(msg.data[14] == pytest.approx(.091) for msg in valid_traces)
        assert all(msg.data[15] == 0.0 for msg in valid_traces)
        valid_duties = [msg for msg in duty_commands if msg.data[2] == 1.0]
        assert valid_duties
        assert all(msg.data[0] == pytest.approx(.091) for msg in valid_duties)
        assert all(msg.data[2] == pytest.approx(2.0) for msg in valid_traces)
        assert all(event['frame_id'] == upper['path_frame_id'] for event in physical_plans)
        assert all(msg.path.header.frame_id == upper['path_frame_id'] for msg in references)
        names = set(probe.get_node_names())
        assert {'upper_planner_node', 'lower_tracking_mpc_node'} <= names
        assert not {'model_real_bridge', 'knu_global_path_publisher', 'px4_ekf_bridge',
                    'px4_odom_map_bridge'} & names
        metrics = {
            'centred_bev': centred_bev, 'extent_m': extent, 'target_speed_mps': target_speed,
            'published_physical_plans': len(physical_plans),
            'max_planning_wall_ms': max(
                1000 * (event['t_emit'] - event['t_start']) for event in physical_plans),
            'max_horizon': max(event['horizon'] for event in physical_plans),
        }
        (tmp_path / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
        # Keep BEV/path fresh, stop only Motive: lower must independently select neutral.
        for _ in range(100):
            image.header.stamp = probe.get_clock().now().to_msg()
            grid_pub.publish(image)
            path_pub.publish(external_path)
            rclpy.spin_once(probe, timeout_sec=0.01)
            time.sleep(0.01)
        assert traces[-1].data[10] == 0.0
        assert traces[-1].data[13] == 0.0
        assert traces[-1].data[14] == 0.0
        assert duty_commands[-1].data[0] == 0.0
        # Fresh Motive and a refreshed path must not release a real-vehicle stop latch.
        for _ in range(50):
            odom.header.stamp = probe.get_clock().now().to_msg()
            odom_pub.publish(odom)
            image.header.stamp = odom.header.stamp
            grid_pub.publish(image)
            path_pub.publish(external_path)
            rclpy.spin_once(probe, timeout_sec=0.01)
            time.sleep(0.01)
        assert traces[-1].data[14] == 0.0
        assert traces[-1].data[12] == 4.0
        assert duty_commands[-1].data[0] == 0.0
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=10)
        if log is not None:
            log.close()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
    # Read the closed recording: a healthy live graph alone does not prove that
    # failure-time BEV and control commands can be recovered after shutdown.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    recordings = list((tmp_path / 'recordings').iterdir())
    assert len(recordings) == 1
    recording = recordings[0]
    metadata = yaml.safe_load((recording / 'bag/metadata.yaml').read_text())
    counts = {row['topic_metadata']['name']: row['message_count']
              for row in metadata['rosbag2_bagfile_information']['topics_with_message_count']}
    for topic in (state_topic, upper['grid_map_topic'], upper['global_path_topic'],
                  '/planner/lower_reference_path/with_arclength',
                  '/debug/upper_preview_reference', '/debug/lower_mpc_trace',
                  '/model/cmd_control', '/model/control/duty_cmd'):
        assert counts.get(topic, 0) > 0, f'not saved: {topic}'
    run = json.loads((recording / 'run.json').read_text())
    assert run['hardware_output'] is False
    assert run['lower_parameters']['mavlink_enable'] is False
    assert (recording / 'node_logs').is_dir()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(recording / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    saved_grid = None
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic == upper['grid_map_topic']:
            saved_grid = deserialize_message(serialized, OccupancyGrid)
            assert received_ns > 0
            break
    assert saved_grid is not None
    assert saved_grid.info == image.info
    assert saved_grid.data == image.data
