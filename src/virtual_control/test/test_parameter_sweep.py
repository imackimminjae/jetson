"""Offline sweep ownership, frozen configuration and continuation checks."""
import copy
import csv
import json
from pathlib import Path
import signal
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
import parameter_sweep as sweep
from repeat_drive_session import effective_config


def prepared(tmp_path, sets=('baseline', 'heading_strong')):
    directory = tmp_path/'sweep'
    plan = sweep.prepare(directory, ['scenario1'], 3, sets)
    return directory, plan


def session_result(directory, runs=3, outcome='completed', phase='done', stopped=True):
    directory = Path(directory)
    results = directory/'results'/'test'
    results.mkdir(parents=True)
    (directory/'session_result.json').write_text(json.dumps({'status': {'phase': phase}, 'failure': ''}))
    rows = [dict(outcome=outcome, duration_sec=40, progress_ratio=.95, cross_track_m=.2,
                 stop_status='stationary' if stopped else 'unknown') for _ in range(runs)]
    with (results/'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def test_default_plan_is_frozen_27_attempts_with_matched_prediction_time(tmp_path):
    directory = tmp_path/'default'
    plan = sweep.prepare(directory, ['scenario1'], 3)
    assert plan['planned_attempts'] == 27 and len(plan['jobs']) == 9
    jobs = {j['set']: j for j in plan['jobs']}
    assert jobs['baseline']['planner_hz'] == 5 and jobs['baseline']['controller_hz'] == 10
    assert jobs['controller_20_horizon3']['controller_preview_sec'] == 3
    cfg = yaml.safe_load(Path(jobs['controller_20_horizon3']['config_file']).read_text())
    assert cfg['lower_tracking_mpc_node']['ros__parameters']['lower_prediction_steps'] == 60
    assert cfg['upper_planner_node']['ros__parameters']['upper_preview_steps'] == 7
    for job in plan['jobs']:
        config = yaml.safe_load(Path(job['config_file']).read_text())
        sweep.validate_config(config)
        p = config['knu_batch_runner']['ros__parameters']
        assert p['routes'] == ['scenario1'] and p['repeat_count'] == 3
        assert p['stationary_confirm_sec'] == .5 and p['map_settle_sec'] == .3
        assert p['reference_min_wait_sec'] == .3
    sweep.verify_plan(plan)


def test_frozen_config_does_not_reload_changed_live_base(tmp_path, monkeypatch):
    directory, plan = prepared(tmp_path)
    path = Path(plan['jobs'][0]['config_file'])
    original_read = Path.read_text

    def read_text(self, *args, **kwargs):
        if self.name == 'tracking_control_split.yaml':
            raise AssertionError('prepared session must not reload live tuning')
        return original_read(self, *args, **kwargs)

    monkeypatch.setattr(Path, 'read_text', read_text)
    cfg = effective_config(3, tmp_path/'results', 'scenario3', guarded=True, config_file=path)
    assert cfg['upper_planner_node']['ros__parameters']['upper_planner_rate_hz'] == 5
    assert cfg['knu_batch_runner']['ros__parameters']['routes'] == ['scenario3']
    assert cfg['knu_batch_runner']['ros__parameters']['results_directory'] == str(tmp_path/'results')


@pytest.mark.parametrize('change', [
    {'lower_tracking_mpc_node': {'lower_prediction_dt_sec': .05}},
    {'knu_batch_runner': {'stationary_confirm_sec': .1}},
    {'upper_planner_node': {'upper_preview_steps': 12}},
    {'upper_planner_node': {'max_steering_angle_rad': .5}},
])
def test_invalid_model_or_transition_settings_rejected(tmp_path, change):
    directory, plan = prepared(tmp_path)
    cfg = yaml.safe_load(Path(plan['jobs'][0]['config_file']).read_text())
    sweep.apply_changes(cfg, change)
    with pytest.raises(ValueError):sweep.validate_config(cfg)


def test_changed_prepared_config_or_route_blocks_execution(tmp_path):
    directory, plan = prepared(tmp_path)
    route = next(k for k in plan['dependencies_sha256'] if k.endswith('scenario1.csv'))
    changed = copy.deepcopy(plan);changed['dependencies_sha256'][route] = 'wrong'
    with pytest.raises(ValueError, match='변경'):sweep.verify_plan(changed)
    path = Path(plan['jobs'][0]['config_file']);path.write_text(path.read_text()+'\n')
    with pytest.raises(ValueError, match='설정이 변경'):sweep.verify_plan(plan)


@pytest.mark.parametrize('outcome,phase,stopped,okay', [
    ('completed', 'done', True, True),
    ('off_route', 'done', True, True),
    ('stopped_10s', 'done', True, True),
    ('grid_timeout', 'done', True, False),
    ('skipped_system_fault', 'done', True, False),
    ('completed', 'aborted', True, False),
    ('completed', 'done', False, False),
])
def test_only_confirmed_stopped_sessions_continue(tmp_path, outcome, phase, stopped, okay):
    session_result(tmp_path/'session', outcome=outcome, phase=phase, stopped=stopped)
    assert sweep.inspect_session(tmp_path/'session', 3)[0] is okay


def test_all_jobs_run_in_order_and_resume_skips_completed_jobs(tmp_path, monkeypatch):
    directory, plan = prepared(tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    calls = []

    class Child:
        def __init__(self, command, **kwargs):
            calls.append(command)
            assert '--guarded' in command and '--config-file' in command
            session_result(command[command.index('--session-dir')+1], outcome='off_route')

        def wait(self):return 0

    monkeypatch.setattr(sweep.subprocess, 'Popen', Child)
    assert sweep.execute(directory) == 0 and len(calls) == 2
    assert sweep.read_json(directory/'progress.json')['state'] == 'complete'
    assert len(list(csv.DictReader((directory/'summary.csv').open()))) == 6
    assert sweep.execute(directory) == 0 and len(calls) == 2


def test_system_fault_stops_before_next_set_and_resume_uses_new_directory(tmp_path, monkeypatch):
    directory, plan = prepared(tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    calls = []

    class Child:
        def __init__(self, command, **kwargs):
            calls.append(command)
            session_result(command[command.index('--session-dir')+1],
                           outcome='grid_timeout' if len(calls) == 1 else 'completed')

        def wait(self):return 0

    monkeypatch.setattr(sweep.subprocess, 'Popen', Child)
    assert sweep.execute(directory) == 1 and len(calls) == 1
    assert sweep.read_json(directory/'progress.json')['state'] == 'blocked'
    assert sweep.execute(directory) == 0 and len(calls) == 3
    assert calls[1][calls[1].index('--session-dir')+1].endswith('_retry02')


def test_interrupt_is_forwarded_and_never_advances(tmp_path, monkeypatch):
    directory, _ = prepared(tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    calls = [];sent = []

    class Child:
        def __init__(self, command, **kwargs):
            calls.append(command);self.session = command[command.index('--session-dir')+1]

        def poll(self):return None
        def send_signal(self, value):sent.append(value)

        def wait(self):
            signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
            session_result(self.session, phase='aborted')
            return 1

    monkeypatch.setattr(sweep.subprocess, 'Popen', Child)
    assert sweep.execute(directory) == 130 and len(calls) == 1
    assert sent == [signal.SIGINT]
    assert not sweep.read_json(directory/'progress.json')['completed']


def test_default_cli_prepares_without_ros_or_children(tmp_path, monkeypatch):
    monkeypatch.setattr(sweep.subprocess, 'Popen', lambda *a, **k: pytest.fail('Must not start a child'))
    assert sweep.main(['--output', str(tmp_path/'offline')]) == 0
    assert sweep.read_json(tmp_path/'offline'/'progress.json')['state'] == 'prepared'
