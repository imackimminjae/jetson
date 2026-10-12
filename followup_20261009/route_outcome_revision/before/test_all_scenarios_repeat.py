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


@pytest.fixture
def supervisor(tmp_path, monkeypatch):
    if os.environ.get('KNU_BATCH_ROS_TEST') != '1':
        pytest.skip('isolated ROS opt-in required')
    assert os.environ.get('ROS_DOMAIN_ID') == '177'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from rclpy.parameter import Parameter
    import knu_batch_runner as module
    clock = [100.0]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    rclpy.init()
    runner = module.KnuBatchRunner(parameter_overrides=[
        Parameter('routes', value=list(SCENARIOS)),
        Parameter('repeat_count', value=5),
        Parameter('repeat_per_route', value=True),
        Parameter('skip_system_fault_scenarios', value=True),
        Parameter('require_compute_heartbeat', value=True),
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
