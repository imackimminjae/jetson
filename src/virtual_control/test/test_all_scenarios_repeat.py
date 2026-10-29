"""Accelerated supervisor checks on isolated ROS; no vehicle-facing nodes."""
import json
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from repeat_drive_session import SCENARIOS, effective_config


def test_directory_size_tolerates_atomic_summary_replacement(tmp_path):
    from repeat_drive_session import directory_size
    kept = tmp_path/'summary.csv'
    kept.write_bytes(b'12345')
    directory = tmp_path/'bag'
    directory.mkdir()

    class ReplacedEntry:
        def is_file(self):
            return True

        def stat(self):
            raise FileNotFoundError('summary.csv.tmp was renamed')

    root = SimpleNamespace(rglob=lambda pattern: iter([kept, ReplacedEntry(), directory]))
    assert directory_size(root) == 5

    class InaccessibleEntry(ReplacedEntry):
        def stat(self):
            raise PermissionError('unexpected access failure')

    root = SimpleNamespace(rglob=lambda pattern: iter([InaccessibleEntry()]))
    with pytest.raises(PermissionError):
        directory_size(root)


def test_runtime_ros_logs_default_and_separate_experiments(tmp_path):
    from repeat_drive_session import runtime_ros_log_directory
    first = tmp_path/'experiment1'/'sessions'/'same_session_name'
    second = tmp_path/'experiment2'/'sessions'/'same_session_name'
    assert runtime_ros_log_directory(first, {}) == first/'ros_logs'
    environment = {'DRIVE_RUNTIME_ROS_LOG_ROOT': str(tmp_path/'runtime')}
    a = runtime_ros_log_directory(first, environment)
    b = runtime_ros_log_directory(second, environment)
    assert a != b
    assert a == tmp_path/'runtime'/'experiment1'/'same_session_name'


def test_recorder_qos_override_preserves_all_recorded_topics(tmp_path, monkeypatch):
    from drive_debug_profile import TOPICS, recorder_command
    monkeypatch.delenv('DRIVE_DEBUG_QOS_OVERRIDES', raising=False)
    ordinary = recorder_command(tmp_path/'bag')
    assert ordinary[-len(TOPICS):] == list(TOPICS)
    profile = tmp_path/'recorder_qos.yaml'
    monkeypatch.setenv('DRIVE_DEBUG_QOS_OVERRIDES', str(profile))
    changed = recorder_command(tmp_path/'bag')
    index = changed.index('--qos-profile-overrides-path')
    assert changed[index+1] == str(profile)
    assert changed[:index]+changed[index+2:] == ordinary


def test_profile_selection_and_count_preserve_controller_parameters(tmp_path):
    assert SCENARIOS == tuple(f'scenario{i}' for i in range(1, 13))
    all_config = effective_config(5, tmp_path, 'all')
    single = effective_config(5, tmp_path, 'scenario1')
    for name in all_config.keys()-{'knu_batch_runner'}:
        assert all_config[name] == single[name]
    p = all_config['knu_batch_runner']['ros__parameters']
    assert p['routes'] == list(SCENARIOS) and p['repeat_count'] == 5
    assert p['repeat_per_route'] and p['skip_system_fault_scenarios']
    assert p['require_compute_heartbeat']
    with pytest.raises(ValueError):
        effective_config(5, tmp_path, 'scenario13')


def test_shortened_route_snapshot_and_completion_use_same_endpoint(tmp_path):
    from repeat_drive_session import snapshot_effective_routes
    from knu_global_path_publisher import load_route
    from knu_batch_monitor import RouteMonitor
    directory = PACKAGE/'data/knu_routes'
    snapshot_effective_routes(['scenario11'], directory, tmp_path, 6)
    prefix = load_route('scenario11', tmp_path)
    original = load_route('scenario11', directory)
    assert prefix == original[:6]
    monitor = RouteMonitor(prefix, 0)
    full_monitor = RouteMonitor(original, 0)
    elapsed = 0.0
    for a, b in zip(prefix, prefix[1:]):
        length = math.dist(a[:2], b[:2])
        steps = math.ceil(length)
        for index in range(steps):
            u = index/steps
            position = tuple(a[k] + u*(b[k]-a[k]) for k in range(2))
            monitor.update(elapsed, position)
            full_monitor.update(elapsed, position)
            elapsed += length/steps/6
    assert monitor.update(elapsed, prefix[-1][:2]) == 'completed'
    assert full_monitor.update(elapsed, prefix[-1][:2]) is None


def test_invalid_prefix_is_rejected_before_session_start(tmp_path):
    import yaml
    config = effective_config(1, tmp_path, 'scenario11', guarded=True)
    config['knu_batch_runner']['ros__parameters']['route_end_waypoint'] = 99
    frozen = tmp_path/'invalid_prefix.yaml'
    frozen.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match='end_waypoint'):
        effective_config(1, tmp_path, 'scenario11', guarded=True, config_file=frozen)


@pytest.fixture
def supervisor(tmp_path, monkeypatch, request):
    if os.environ.get('KNU_BATCH_ROS_TEST') != '1':
        pytest.skip('isolated ROS opt-in required')
    assert os.environ.get('ROS_DOMAIN_ID') == '177'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from rclpy.parameter import Parameter
    import knu_batch_runner as module
    clock = [100.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    scenario = getattr(request, 'param', 'all')
    config = effective_config(5, tmp_path, scenario, guarded=True)
    settings = config['knu_batch_runner']['ros__parameters']
    rclpy.init()
    runner = module.KnuBatchRunner(parameter_overrides=[
        *[Parameter(name, value=settings[name]) for name in (
            'routes', 'repeat_count', 'repeat_per_route',
            'skip_system_fault_scenarios', 'require_compute_heartbeat',
            'upper_compute_timeout_sec', 'lower_compute_timeout_sec')],
        Parameter('results_directory', value=str(tmp_path))])
    runner.timer.cancel()
    permissions = []
    monkeypatch.setattr(runner, 'publish_permission', permissions.append)

    def step(dt=0.25, speed=0.0, pose=True, grid=True, upper=True, lower=True):
        clock[0] += dt
        if pose:
            runner.pose = (*runner.start_positions[runner.route][:2], speed)
            runner.pose_at = clock[0]
        if grid:
            runner.grid_at = clock[0]
        if upper:
            runner.upper_cycle_at = clock[0]
        if lower:
            runner.lower_cycle_at = clock[0]
        runner.tick()

    runner.next_route()
    try:
        yield SimpleNamespace(r=runner, step=step, clock=clock, permissions=permissions)
    finally:
        runner.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('scenario', ['scenario7', 'scenario8', 'scenario12',
                                     'scenario10', 'scenario11'])
def test_guarded_single_preserves_route_and_controller_settings(tmp_path, scenario):
    legacy = effective_config(5, tmp_path, scenario)
    guarded = effective_config(5, tmp_path, scenario, guarded=True)
    for node in legacy.keys()-{'knu_batch_runner'}:
        assert legacy[node] == guarded[node]
    original = legacy['knu_batch_runner']['ros__parameters']
    selected = guarded['knu_batch_runner']['ros__parameters']
    monitoring_keys = {'repeat_per_route', 'skip_system_fault_scenarios',
                       'require_compute_heartbeat', 'upper_compute_timeout_sec',
                       'lower_compute_timeout_sec'}
    assert {k: v for k, v in original.items() if k not in monitoring_keys} == {
        k: v for k, v in selected.items() if k not in monitoring_keys}
    assert selected['routes'] == [scenario] and selected['repeat_count'] == 5
    assert selected['require_compute_heartbeat'] and selected['skip_system_fault_scenarios']
    assert selected['upper_compute_timeout_sec'] == 5.0
    assert selected['lower_compute_timeout_sec'] == 2.0
    assert not original.get('require_compute_heartbeat', False)


@pytest.mark.parametrize('supervisor', ['scenario7'], indirect=True)
@pytest.mark.parametrize('level,timeout', [('upper', 5.0), ('lower', 2.0)])
def test_guarded_single_waits_for_stop_and_recovery_before_skipping(supervisor, level, timeout):
    h = supervisor
    assert h.r.routes == ['scenario7']*5
    h.r.set_phase('running')
    h.r.upper_cycle_at = h.r.lower_cycle_at = h.clock[0]
    h.step(dt=timeout+0.01, **{level: False})
    assert h.r.rows[0]['outcome'] == level+'_compute_timeout'
    assert h.permissions[-1] is False
    for _ in range(8):
        h.step(speed=1.0, **{level: False})
        assert h.r.index == 0 and len(h.r.rows) == 1
    for _ in range(8):
        h.step(**{level: False})
        assert h.r.index == 0 and len(h.r.rows) == 1
    h.step()
    assert h.r.phase == 'done'
    assert len(h.r.rows) == 5 and h.r.rows[0]['stop_status'] == 'stationary'
    assert [r['outcome'] for r in h.r.rows[1:]] == ['skipped_system_fault']*4
    assert not any(h.permissions)


@pytest.mark.parametrize('supervisor', ['scenario7'], indirect=True)
def test_guarded_single_driving_failures_continue_all_five(supervisor):
    h = supervisor
    for index in range(5):
        assert h.r.index == index and h.r.route == 'scenario7'
        h.r.set_phase('running')
        h.r.fail_or_finish('off_route')
        for _ in range(5):
            h.step()
        assert h.r.rows[-1]['stop_status'] == 'stationary'
    assert h.r.phase == 'done' and len(h.r.rows) == 5
    assert not h.r.system_fault_scenarios
    assert not any(h.permissions)


@pytest.mark.parametrize('supervisor', ['scenario1'], indirect=True)
def test_short_stationary_confirmation_still_restarts_after_motion(supervisor):
    h = supervisor
    h.r.settings['stationary_confirm_sec'] = .5
    h.step(dt=.1)
    h.step(dt=.49)
    assert h.r.phase == 'stopping'
    h.step(dt=.02, speed=1.0)
    assert h.r.stationary_since is None
    h.step(dt=.1)
    h.step(dt=.49)
    assert h.r.phase == 'stopping'
    h.step(dt=.02)
    assert h.r.phase == 'pausing'


@pytest.mark.parametrize('supervisor', ['scenario1'], indirect=True)
def test_short_map_wait_still_requires_a_new_grid_after_alignment(supervisor):
    h = supervisor
    h.r.settings['map_settle_sec'] = .3
    h.r.settings['reset_upper_each_attempt'] = True
    h.r.set_phase('map')
    h.r.map_pose_ready_at = None
    h.step(dt=.1)
    h.step(dt=.31, grid=False)
    assert h.r.phase == 'map'
    h.step(dt=.01)
    assert h.r.phase == 'resetting'


@pytest.mark.parametrize('supervisor', ['scenario1'], indirect=True)
def test_short_reference_wait_still_requires_a_fresh_reference(supervisor):
    h = supervisor
    h.r.settings['reference_min_wait_sec'] = .3
    h.r.set_phase('reference')
    h.r.reference_at = h.clock[0] + .25
    h.step(dt=.25)
    assert h.r.phase == 'reference'
    h.step(dt=.06)
    assert h.r.phase == 'running'


def test_driving_failures_still_attempt_all_sixty_slots(supervisor):
    h = supervisor
    outcomes = ['completed', 'off_route', 'stopped_10s', 'no_progress', 'route_timeout']
    for index in range(60):
        assert h.r.index == index and h.r.route == f'scenario{index//5+1}'
        h.r.set_phase('running')
        h.r.fail_or_finish(outcomes[index % 5])
        for _ in range(5):
            h.step(speed=1.0)
            assert h.r.index == index and h.r.phase == 'coasting'
        for _ in range(5):
            h.step()
        assert h.r.rows[-1]['stop_status'] == 'stationary'
    assert h.r.phase == 'done' and len(h.r.rows) == 60
    assert not h.r.system_fault_scenarios
    assert not any(h.permissions)


@pytest.mark.parametrize('outcome', ['pose_timeout', 'grid_timeout', 'setup_timeout',
                                    'alignment_failed', 'upper_compute_timeout',
                                    'lower_compute_timeout'])
def test_only_faulted_scenario_skipped_after_stop_and_recovery(supervisor, outcome):
    h = supervisor
    h.r.fail_or_finish(outcome)
    for _ in range(8):
        h.step(speed=1.0)
        assert h.r.index == 0
    for _ in range(5):
        h.step()
    assert h.r.index == 5 and h.r.route == 'scenario2'
    assert h.r.rows[0]['outcome'] == outcome
    assert h.r.rows[0]['stop_status'] == 'stationary'
    assert [r['outcome'] for r in h.r.rows[1:]] == ['skipped_system_fault']*4
    assert all(r['duration_sec'] == 0 for r in h.r.rows[1:])
    assert len({r['token'] for r in h.r.rows}) == 5
    assert not any(h.permissions)


@pytest.mark.parametrize('level,timeout', [('upper', 5.0), ('lower', 2.0)])
def test_completion_watchdog_stops_then_waits_for_recovery(supervisor, level, timeout):
    h = supervisor
    h.r.set_phase('running')
    h.r.upper_cycle_at = h.r.lower_cycle_at = h.clock[0]
    h.step(dt=timeout+0.01, **{level: False})
    assert h.r.phase == 'coasting'
    assert h.r.rows[0]['outcome'] == level+'_compute_timeout'
    assert h.permissions[-1] is False
    for _ in range(8):
        h.step(**{level: False})
        assert h.r.index == 0
    h.step()
    assert h.r.index == 5


@pytest.mark.parametrize('missing', ['pose', 'grid', 'upper', 'lower'])
def test_persistent_system_loss_aborts_instead_of_burning_next_scenarios(supervisor, missing):
    h = supervisor
    h.r.fail_or_finish('grid_timeout')
    for _ in range(65):
        h.step(dt=1.0, **{missing: False})
    assert h.r.phase == 'aborted' and h.r.index == 0
    assert len(h.r.rows) == 1
    assert not any(h.permissions)


def test_failed_solver_cycles_are_heartbeats_but_duplicate_diagnostics_are_not(supervisor):
    from std_msgs.msg import String, Float64MultiArray
    h = supervisor
    h.r.on_upper_cycle(String(data=json.dumps({'cycle_seq': 1, 'outcome': 'solver_rejected'})))
    trace = [0.0]*32
    trace[0] = 100.0  # qp_valid remains zero, including neutral/failure.
    h.r.on_lower_cycle(Float64MultiArray(data=trace))
    assert h.r.compute_issue(h.clock[0]) is None
    h.clock[0] += 6.0
    h.r.on_upper_cycle(String(data=json.dumps({'cycle_seq': 1})))
    h.r.on_lower_cycle(Float64MultiArray(data=trace))
    assert h.r.compute_issue(h.clock[0]) == 'upper_compute_timeout'
    assert math.isfinite(h.r.upper_cycle_at)


def test_late_alignment_cannot_reanchor_after_next_scenario_start(supervisor):
    h = supervisor
    done = [False]
    h.r.future = SimpleNamespace(done=lambda: done[0])
    h.r.fail_or_finish('setup_timeout', 'aligning')
    for _ in range(8):
        h.step()
        assert h.r.index == 0
    done[0] = True
    h.step()
    assert h.r.index == 5 and h.r.future is None


def test_compute_stall_during_coasting_attributed_to_original_scenario(supervisor):
    h = supervisor
    h.r.fail_or_finish('off_route')
    for _ in range(8):
        h.step(lower=False)
    assert h.r.index == 0
    assert h.r.rows[0]['outcome'] == 'off_route'
    assert 'lower_compute_timeout during coasting' in h.r.rows[0]['detail']
    h.step()
    assert h.r.index == 5


@pytest.mark.parametrize('level', ['upper', 'lower'])
def test_long_completed_solve_not_hidden_by_fresh_diagnostic(supervisor, level):
    from std_msgs.msg import String, Float64MultiArray
    h = supervisor
    h.r.set_phase('running')
    h.r.upper_cycle_at = h.r.lower_cycle_at = h.clock[0]
    if level == 'upper':
        h.r.on_upper_cycle(String(data=json.dumps({'cycle_seq': 1, 'wall_ms': 6000.0})))
    else:
        trace = [0.0]*32
        trace[0], trace[11] = 100.0, 2500.0
        h.r.on_lower_cycle(Float64MultiArray(data=trace))
    h.step()
    assert h.r.rows[0]['outcome'] == level+'_compute_timeout'
    assert h.r.phase == 'coasting'
    for _ in range(5):
        h.step()
    assert h.r.index == 5


@pytest.mark.parametrize('stamped',[True,False])
def test_route_unavailable_stops_and_continues_next_repetition(supervisor,stamped):
    from knu_batch_monitor import RouteMonitor, RoadRouteCheck
    from nav_msgs.msg import Odometry, OccupancyGrid
    h = supervisor
    h.r.set_phase('running')
    h.r.start_positions['scenario1'] = (0.,0.,0.)
    h.r.monitor = RouteMonitor([(0,0),(100,0)],h.clock[0])
    h.r.road_check = RoadRouteCheck([(0,0),(100,0)])
    g = OccupancyGrid()
    g.header.frame_id = 'Vehicle'
    g.info.width,g.info.height,g.info.resolution = 80,40,.5
    g.info.origin.position.y = -10.
    g.info.origin.orientation.w = 1.
    g.data = [0 if x < 20 else 100 for y in range(40) for x in range(80)]
    for i in range(14):
        od = Odometry();od.header.frame_id = 'map'
        od.header.stamp.sec = int(h.clock[0])
        od.header.stamp.nanosec = int((h.clock[0]%1)*1e9)
        if stamped:
            g.header.stamp = od.header.stamp
        h.r.on_pose(od)
        h.r.on_grid(g)
        h.step(dt=.25)
        if h.r.phase == 'coasting': break
    assert h.r.phase == 'coasting'
    assert h.r.rows[0]['outcome'] == 'route_unavailable'
    assert json.loads(h.r.rows[0]['detail'])['road_evidence']['confirmed']
    assert json.loads(h.r.rows[0]['detail'])['road_evidence']['acquisition_stamp_available']==stamped
    assert h.permissions[-1] is False
    assert not h.r.system_fault_scenarios
    for _ in range(100):
        h.step()
        if h.r.index != 0: break
    assert h.r.index == 1
    assert h.r.rows[0]['stop_status'] == 'stationary'
    assert h.r.road_check is None


def test_duplicate_stamped_grids_cannot_confirm(supervisor):
    from knu_batch_monitor import RouteMonitor,RoadRouteCheck
    from nav_msgs.msg import Odometry,OccupancyGrid
    h=supervisor
    h.r.set_phase('running')
    h.r.start_positions['scenario1']=(0.,0.,0.)
    h.r.monitor=RouteMonitor([(0,0),(100,0)],h.clock[0])
    h.r.road_check=RoadRouteCheck([(0,0),(100,0)])
    g=OccupancyGrid();g.header.frame_id='Vehicle';g.header.stamp.sec=100
    g.info.width,g.info.height,g.info.resolution=80,40,.5
    g.info.origin.position.y=-10.;g.info.origin.orientation.w=1.
    g.data=[0 if x<20 else 100 for y in range(40) for x in range(80)]
    for _ in range(16):
        od=Odometry();od.header.frame_id='map';od.header.stamp=g.header.stamp
        h.r.on_pose(od);h.r.on_grid(g);h.step(dt=.25)
    assert h.r.phase=='running' and not h.r.road_check.confirmed(h.clock[0])


def test_unknown_map_keeps_original_stop_with_evidence_detail(supervisor):
    from knu_batch_monitor import RouteMonitor,RoadRouteCheck,Limits
    h=supervisor
    h.r.set_phase('running')
    h.r.start_positions['scenario1']=(0.,0.,0.)
    h.r.monitor=RouteMonitor([(0,0),(100,0)],h.clock[0],Limits(stop_sec=1))
    h.r.road_check=RoadRouteCheck([(0,0),(100,0)])
    h.r.road_check.observe(h.clock[0],1,{'state':'unavailable','reason':'unknown samples'})
    for _ in range(4):h.step()
    assert h.r.rows[0]['outcome']=='stopped_10s'
    detail=json.loads(h.r.rows[0]['detail'])
    assert detail['waypoint_guard_result']=='stopped_10s'
    assert detail['road_evidence']['state']=='unavailable'


def test_road_evidence_cannot_override_compute_stall(supervisor):
    from knu_batch_monitor import RouteMonitor, RoadRouteCheck
    h=supervisor
    h.r.set_phase('running')
    h.r.monitor=RouteMonitor([(0,0),(100,0)],h.clock[0])
    h.r.road_check=RoadRouteCheck([(0,0),(100,0)])
    e=dict(state='route_unavailable',reason='synthetic',blocked_start_m=10,blocked_end_m=15)
    for i in range(16): h.r.road_check.observe(h.clock[0]-3+i*.2,i,e)
    assert h.r.road_check.confirmed(h.clock[0])
    h.r.compute_overrun='lower_compute_timeout'
    h.step()
    assert h.r.rows[0]['outcome']=='lower_compute_timeout'


def test_effective_evidence_grid_settings_match_planner(tmp_path):
    config=effective_config(5,tmp_path,'all')
    r=config['knu_batch_runner']['ros__parameters']
    u=config['upper_planner_node']['ros__parameters']
    assert r['road_grid_positive_is_drivable']==u['grid_positive_is_drivable']
    assert r['road_grid_value_threshold']==u['grid_value_threshold']
    assert r['road_expected_grid_frame']==u['expected_grid_frame_id']
    assert r['road_enable_waypoint_bias'] is False
    assert r['road_waypoint_yaw_bias_rad']==0


@pytest.mark.parametrize('angle', [-10, -5, 0, 5, 10])
def test_bias_changes_planning_input_without_moving_outcome_route(tmp_path, angle):
    import yaml
    base=effective_config(1,tmp_path,'scenario4',guarded=True)
    upper=base['upper_planner_node']['ros__parameters']
    upper.update(enable_waypoint_bias=True,waypoint_yaw_bias_rad=math.radians(angle),
                 waypoint_x_bias_m=1.0,waypoint_y_bias_m=-2.0)
    # A previously frozen configuration must not restore biased road evidence.
    base['knu_batch_runner']['ros__parameters'].update(
        road_enable_waypoint_bias=True,road_waypoint_yaw_bias_rad=math.radians(angle),
        road_waypoint_x_bias_m=1.0,road_waypoint_y_bias_m=-2.0)
    frozen=tmp_path/'biased.yaml';frozen.write_text(yaml.safe_dump(base))
    config=effective_config(1,tmp_path,'scenario4',guarded=True,config_file=frozen)
    assert config['upper_planner_node']['ros__parameters']==upper
    assert config['lower_tracking_mpc_node']==base['lower_tracking_mpc_node']
    runner=config['knu_batch_runner']['ros__parameters']
    assert runner['road_enable_waypoint_bias'] is False
    assert all(runner[k]==0 for k in ('road_waypoint_yaw_bias_rad','road_waypoint_x_bias_m','road_waypoint_y_bias_m'))
    for key in ('goal_radius_m','finish_remaining_m','min_progress_ratio'):
        assert runner[key]==base['knu_batch_runner']['ros__parameters'][key]


@pytest.mark.parametrize('supervisor', ['scenario4'], indirect=True)
@pytest.mark.parametrize('angle', [-10, 10])
def test_runner_ignores_legacy_bias_for_road_and_completion(supervisor, angle):
    h=supervisor
    h.r.settings.update(enable_road_route_check=True,road_enable_waypoint_bias=True,
                        road_waypoint_yaw_bias_rad=math.radians(angle),
                        road_waypoint_x_bias_m=1.0,road_waypoint_y_bias_m=-2.0)
    mission=[tuple(p[:2]) for p in h.r.points[h.r.route]]
    h.r.set_phase('reference')
    h.r.reference_at=h.clock[0]+.3
    h.step(dt=1.1)
    assert h.r.phase=='running'
    assert h.r.monitor.points==mission
    assert h.r.road_check.route.points==mission
    assert h.r.monitor.points[-1]==mission[-1]
