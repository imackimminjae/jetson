"""Offline recorder/config checks; never start vehicle-facing nodes."""
from pathlib import Path
import sqlite3
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from drive_debug_profile import TOPICS, recorder_command
from repeat_drive_session import effective_config, resource_error, MIB


@pytest.mark.parametrize('scenario', ['scenario1', 'scenario2', 'scenario3'])
def test_effective_configuration_preserves_manual_udp_and_tuning(tmp_path, scenario):
    import yaml
    base = yaml.safe_load((PACKAGE/'config/tracking_control_split.yaml').read_text())
    config = effective_config(10, tmp_path, scenario)
    bridge = config['px4_ekf_bridge']['ros__parameters']
    lower = config['lower_tracking_mpc_node']['ros__parameters']
    upper = config['upper_planner_node']['ros__parameters']
    assert bridge['connection_url'] == 'udpin:0.0.0.0:14541'
    assert bridge['configure_px4_parameters'] is False
    assert lower['mavlink_bind_port'] == 14540
    assert all(lower[key] is False for key in
               ('mavlink_auto_manual_mode', 'mavlink_auto_arm', 'mavlink_force_arm'))
    for key in ('upper_preview_steps', 'guard_side_boundary_terminal_loss', 'upper_r_dpsi',
                'preview_interval_soft_ratio', 'preview_interval_boundary_margin_m'):
        assert upper[key] == base['upper_planner_node']['ros__parameters'][key]
    assert upper['input_timeout_sec'] == 3.0 and upper['grid_value_threshold'] == 1
    assert config['knu_batch_runner']['ros__parameters']['repeat_count'] == 10
    assert config['knu_batch_runner']['ros__parameters']['require_recorder'] is True
    assert config['knu_batch_runner']['ros__parameters']['routes'] == [scenario]
    assert config['knu_batch_runner']['ros__parameters']['reset_upper_each_attempt'] is True
    assert upper['enable_batch_mission_reset'] is True
    assert config['px4_odom_map_bridge']['ros__parameters']['route_name'] == scenario
    assert config['knu_global_path_publisher']['ros__parameters']['route_name'] == scenario


def test_scenario_selection_changes_only_route_and_keeps_default_compatible(tmp_path):
    default = effective_config(10, tmp_path)
    assert default == effective_config(10, tmp_path, 'scenario3')
    for scenario in ('scenario1', 'scenario2'):
        selected = effective_config(10, tmp_path, scenario)
        selected['knu_batch_runner']['ros__parameters']['routes'] = ['scenario3']
        for name in ('px4_odom_map_bridge', 'knu_global_path_publisher'):
            selected[name]['ros__parameters']['route_name'] = 'scenario3'
        assert selected == default
    with pytest.raises(ValueError, match='Unsupported repeat scenario'):
        effective_config(10, tmp_path, 'scenario99')


def test_resource_limits_and_essential_recording():
    assert not resource_error(1536*MIB, 10*MIB)
    assert resource_error(1024*MIB-1, 10*MIB)
    assert resource_error(1536*MIB, 512*MIB)
    cmd = recorder_command('/tmp/test_bag')
    assert 'mcap' in cmd and '--compression-mode' not in cmd
    assert '/bev/occupancy_grid' in TOPICS and '/debug/upper_branch_event' in TOPICS
    assert '/debug/upper_interval_centering_state' in TOPICS
    assert '/planner/lower_reference_path/with_arclength' in TOPICS
    assert '/px4/ekf_odom' not in TOPICS and '/control/applied_cmd' not in TOPICS


def test_preflight_rejects_existing_controller_before_opening_ports(monkeypatch):
    from types import SimpleNamespace
    import repeat_drive_session as module
    moments = iter((0.0, 4.0))
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: next(moments)))
    node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('lower_tracking_mpc_node', '/')],
        get_name=lambda: 'repeat_drive_session')
    with pytest.raises(RuntimeError, match='기존 5개'):
        module.preflight(node)


def test_recent_bag_essential_messages_lossless_mcap_roundtrip(tmp_path):
    import rosbag2_py
    source = PACKAGE.parents[1]/'drive_debug_20261008_052329_light/bag/bag_0.db3'
    if not source.exists():
        pytest.skip('local reference bag not available')
    connection = sqlite3.connect(f'file:{source}?mode=ro', uri=True)
    topics = {i: (n, t) for i, n, t in connection.execute('select id,name,type from topics')
              if n in TOPICS}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'), storage_id='mcap',
                storage_config_uri=str(PACKAGE/'config/drive_debug_mcap.yaml')),
                rosbag2_py.ConverterOptions('', ''))
    for name, kind in topics.values():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind,
                                                   serialization_format='cdr'))
    rows = [(topics[i][0], data, stamp) for i, stamp, data in connection.execute(
        'select topic_id,timestamp,data from messages order by timestamp') if i in topics]
    for name, data, stamp in rows:
        writer.write(name, data, stamp)
    del writer
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(tmp_path/'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for expected in rows:
        assert reader.read_next() == expected
    assert not reader.has_next()
    total = sum(p.stat().st_size for p in (tmp_path/'bag').iterdir())
    assert total < source.stat().st_size/5
    print(f'{len(rows)} messages identical; original {source.stat().st_size}; essential {total}')
