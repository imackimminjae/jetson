#!/usr/bin/env python3
"""Prepare frozen tuning sets, then run supervised sessions sequentially."""
import argparse
import copy
import csv
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

import yaml
from drive_debug_profile import PACKAGE
from repeat_drive_session import WORKSPACE, SCENARIOS, effective_config

CATALOG = PACKAGE/'config/tuning_sweep.yaml'
WAIT_KEYS = {'stationary_confirm_sec', 'map_settle_sec', 'reference_min_wait_sec'}
TUNING_KEYS = {
    'upper_planner_node': {'upper_planner_rate_hz', 'upper_prediction_dt_sec',
                          'upper_preview_steps', 'lower_prediction_dt_sec',
                          'lambda_relative_turn', 'upper_r_dpsi', 'max_steering_angle_rad'},
    'lower_tracking_mpc_node': {'lower_controller_rate_hz', 'lower_prediction_dt_sec',
                              'lower_prediction_steps', 'lower_q_heading',
                              'lower_rd_steering_rate', 'lower_max_steering_rate_radps'},
    'knu_batch_runner': WAIT_KEYS,
}
SYSTEM_OUTCOMES = {'pose_timeout', 'grid_timeout', 'setup_timeout', 'alignment_failed',
                   'upper_compute_timeout', 'lower_compute_timeout', 'skipped_system_fault',
                   'interrupted'}


def read_json(path):
    return json.loads(Path(path).read_text())


def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def apply_changes(config, changes):
    for node, values in changes.items():
        if node not in ('upper_planner_node', 'lower_tracking_mpc_node', 'knu_batch_runner'):
            raise ValueError(f'Unsupported tuning node: {node}')
        parameters = config[node]['ros__parameters']
        for key, value in values.items():
            if key not in TUNING_KEYS[node]:
                raise ValueError(f'{node}.{key} is not an allowed sweep parameter')
            if key not in parameters and not (node == 'knu_batch_runner' and key in WAIT_KEYS):
                raise ValueError(f'Unknown tuning parameter: {node}.{key}')
            if node == 'knu_batch_runner' and key not in WAIT_KEYS:
                raise ValueError('The tuning catalog may change only runner wait durations')
            # Tuning is numeric; transport, ARM, inputs and coordination stay fixed.
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{node}.{key} must be a finite number')
            if key in parameters:
                original = parameters[key]
                if isinstance(original, bool) or not isinstance(original, (int, float)):
                    raise ValueError(f'{node}.{key} is not a numeric tuning parameter')
                if type(original) is int and type(value) is not int:
                    raise ValueError(f'{node}.{key} requires an integer')
                value = float(value) if type(original) is float else value
            parameters[key] = value


def validate_config(config):
    upper = config['upper_planner_node']['ros__parameters']
    lower = config['lower_tracking_mpc_node']['ros__parameters']
    runner = config['knu_batch_runner']['ros__parameters']
    for p, keys in [(upper, ('upper_planner_rate_hz', 'upper_prediction_dt_sec',
                            'upper_preview_steps', 'max_steering_angle_rad', 'lower_prediction_dt_sec')),
                    (lower, ('lower_controller_rate_hz', 'lower_prediction_dt_sec',
                             'lower_prediction_steps', 'lower_max_steering_angle_rad'))]:
        for key in keys:
            if not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
    for p, keys in [(upper, ('lambda_relative_turn', 'upper_r_dpsi')),
                    (lower, ('lower_q_heading', 'lower_rd_steering_rate',
                             'lower_max_steering_rate_radps'))]:
        if any(not math.isfinite(p[k]) or p[k] < 0 for k in keys):
            raise ValueError('Costs and steering rate limits must be finite and nonnegative')
    if lower['lower_enable_steering_actuator_model'] and not math.isclose(
            1/lower['lower_controller_rate_hz'], lower['lower_prediction_dt_sec'], abs_tol=1e-6):
        raise ValueError('Controller period must equal actuator-model prediction dt')
    if (type(upper['upper_preview_steps']) is not int or upper['upper_preview_steps'] > 7 or
            type(lower['lower_prediction_steps']) is not int or lower['lower_prediction_steps'] > 120):
        raise ValueError('This sweep keeps upper MIQP <=7 steps and lower QP <=120 steps')
    if upper['max_steering_angle_rad'] > lower['lower_max_steering_angle_rad']:
        raise ValueError('Upper steering envelope exceeds the physical lower limit')
    for key, floor in [('stationary_confirm_sec', .5), ('map_settle_sec', .2),
                       ('reference_min_wait_sec', .2)]:
        if not math.isfinite(runner[key]) or runner[key] < floor:
            raise ValueError(f'{key} must be >= {floor}')
    if any(lower[k] for k in ('mavlink_auto_arm', 'mavlink_force_arm', 'mavlink_auto_manual_mode')):
        raise ValueError('Sweep requires the established manual ARM/mode settings')
    if config['px4_ekf_bridge']['ros__parameters']['configure_px4_parameters']:
        raise ValueError('Sweep must not configure PX4 parameters')
    if not all(runner[k] for k in ('require_recorder', 'reset_upper_each_attempt',
                                  'require_compute_heartbeat', 'skip_system_fault_scenarios')):
        raise ValueError('Recorder, per-attempt reset and guarded supervision are required')


def catalog_sets(catalog, selected=None):
    if catalog.get('version') != 1:
        raise ValueError('Unsupported sweep catalog version')
    names = list(selected or catalog['default_sets'])
    if not names or len(names) != len(set(names)):
        raise ValueError('Select a nonempty list without duplicate sets')
    for name in names:
        if name not in catalog['sets'] or not name.replace('_', '').isalnum():
            raise ValueError(f'Unknown or invalid set: {name}')
    return names


def prepare(destination, scenarios, runs, selected=None, catalog_file=CATALOG):
    if type(runs) is not int or not 1 <= runs <= 100:
        raise ValueError('runs must be an integer in [1,100]')
    if not scenarios or len(set(scenarios)) != len(scenarios) or any(s not in SCENARIOS for s in scenarios):
        raise ValueError('Select distinct supported scenarios')
    catalog = yaml.safe_load(Path(catalog_file).read_text())
    names = catalog_sets(catalog, selected)
    destination = Path(destination).resolve()
    base = effective_config(runs, destination/'unused_results', scenarios[0], guarded=True)
    apply_changes(base, catalog['baseline'])
    validate_config(base)
    configurations = []
    for name in names:
        config = copy.deepcopy(base)
        apply_changes(config, catalog['sets'][name]['changes'])
        validate_config(config)
        configurations.append((name, config))
    # Validate every set before leaving a partially prepared sweep directory.
    destination.mkdir(parents=True, exist_ok=False)
    (destination/'configs').mkdir()
    (destination/'base_effective_config.yaml').write_text(yaml.safe_dump(base, sort_keys=False))
    (destination/'catalog.yaml').write_text(yaml.safe_dump(catalog, sort_keys=False, allow_unicode=True))
    jobs = []
    for index, (name, config) in enumerate(configurations, 1):
        for scenario in scenarios:
            job_id = f'{index:02d}_{name}_{scenario}'
            session = destination/'sessions'/job_id
            # Resolve routing against the frozen set, without reloading live base YAML.
            frozen_path = destination/'configs'/f'{job_id}.yaml'
            frozen_path.write_text(yaml.safe_dump(config, sort_keys=False))
            frozen = effective_config(runs, session/'results', scenario, guarded=True,
                                      config_file=frozen_path)
            validate_config(frozen)
            frozen_path.write_text(yaml.safe_dump(frozen, sort_keys=False))
            u = frozen['upper_planner_node']['ros__parameters']
            l = frozen['lower_tracking_mpc_node']['ros__parameters']
            jobs.append(dict(id=job_id, set=name, label=catalog['sets'][name]['label'],
                             scenario=scenario, runs=runs, config_file=str(frozen_path),
                             config_sha256=digest(frozen_path), session_dir=str(session),
                             planner_hz=u['upper_planner_rate_hz'], controller_hz=l['lower_controller_rate_hz'],
                             controller_preview_sec=l['lower_prediction_dt_sec']*l['lower_prediction_steps'],
                             upper_heading_cost=u['lambda_relative_turn'], upper_heading_correction_cost=u['upper_r_dpsi'],
                             upper_steering_limit_rad=u['max_steering_angle_rad'],
                             lower_heading_cost=l['lower_q_heading'], lower_steering_change_cost=l['lower_rd_steering_rate'],
                             lower_steering_rate_radps=l['lower_max_steering_rate_radps']))
    dependencies = [Path(__file__), PACKAGE/'scripts/repeat_drive_session.py',
                    PACKAGE/'scripts/knu_batch_runner.py', PACKAGE/'scripts/knu_batch_monitor.py',
                    PACKAGE/'scripts/drive_debug_profile.py', PACKAGE/'scripts/knu_global_path_publisher.py',
                    PACKAGE/'launch/scenario3_repeat.launch.py', PACKAGE/'config/drive_debug_mcap.yaml',
                    PACKAGE/'data/knu_routes/knu_start_poses.json', PACKAGE/'data/knu_routes/knu_batch_profile.json']
    dependencies += [PACKAGE/f'data/knu_routes/{s}.csv' for s in scenarios]
    dependencies += [WORKSPACE/'install/virtual_control/lib/virtual_control'/name for name in
                     ('upper_planner_node', 'lower_tracking_mpc_node', 'px4_odom_map_bridge', 'px4_ekf_bridge.py')]
    plan = dict(version=1, scenarios=list(scenarios), runs=runs, sets=names,
                planned_attempts=len(jobs)*runs, jobs=jobs,
                dependencies_sha256={str(p.resolve()): digest(p) for p in dependencies},
                stationary_confirm_sec=.5, map_settle_sec=.3, reference_min_wait_sec=.3)
    save_json(destination/'plan.json', plan)
    save_json(destination/'progress.json', {'state': 'prepared', 'completed': {}, 'active': None})
    with (destination/'sets.csv').open('w', newline='') as stream:
        columns = [k for k in jobs[0] if k not in ('config_file', 'config_sha256', 'session_dir')]
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader(); writer.writerows(jobs)
    return plan


def inspect_session(session, runs):
    """Only complete, stopped sessions may advance to another tuning set."""
    session = Path(session)
    result_path = session/'session_result.json'
    if not result_path.exists():
        return False, '세션 종료 기록 없음', []
    result = read_json(result_path)
    if result.get('failure') or result.get('status', {}).get('phase') != 'done':
        return False, result.get('failure') or '세션 중단/미완료', []
    summaries = list(session.glob('results/*/summary.csv'))
    if len(summaries) != 1:
        return False, '회차 결과 파일을 확인할 수 없음', []
    rows = list(csv.DictReader(summaries[0].open()))
    if len(rows) != runs:
        return False, '예정 회차 수와 기록 불일치', rows
    if any(r['outcome'] in SYSTEM_OUTCOMES or r['outcome'].startswith('skipped') for r in rows):
        return False, '입력/계산/준비 이상 또는 회차 생략', rows
    if any(r.get('stop_status') != 'stationary' for r in rows):
        return False, '모든 회차의 정지 확인이 필요함', rows
    return True, '', rows


def verify_plan(plan):
    for path, expected in plan['dependencies_sha256'].items():
        if not Path(path).is_file() or digest(path) != expected:
            raise ValueError(f'준비 이후 실행파일/경로가 변경됨: {path}; 새 계획을 준비하세요')
    for job in plan['jobs']:
        if digest(job['config_file']) != job['config_sha256']:
            raise ValueError(f"준비한 설정이 변경됨: {job['config_file']}")
        validate_config(yaml.safe_load(Path(job['config_file']).read_text()))


def collect_summary(destination, plan, progress):
    rows = []
    for job in plan['jobs']:
        session = progress['completed'].get(job['id'])
        if not session:
            continue
        okay, reason, attempts = inspect_session(session, job['runs'])
        if not okay:
            raise ValueError(f"완료 기록을 다시 확인할 수 없음: {job['id']}: {reason}")
        for index, row in enumerate(attempts, 1):
            rows.append(dict(set=job['set'], label=job['label'], scenario=job['scenario'],
                             attempt=index, planner_hz=job['planner_hz'], controller_hz=job['controller_hz'],
                             controller_preview_sec=job['controller_preview_sec'], outcome=row['outcome'],
                             duration_sec=row['duration_sec'], progress_ratio=row['progress_ratio'],
                             cross_track_m=row['cross_track_m'], stop_status=row['stop_status'], session=session))
    if rows:
        path = Path(destination)/'summary.csv'
        with path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def execute(destination):
    destination = Path(destination).resolve()
    plan = read_json(destination/'plan.json');verify_plan(plan)
    progress = read_json(destination/'progress.json')
    if os.environ.get('ROS_DOMAIN_ID') != '1':
        raise ValueError('실제 반복 시험은 ROS_DOMAIN_ID=1로 실행하세요')
    stopping = False
    child = None
    previous_handlers = {}

    def interrupt(signum, frame):
        nonlocal stopping
        stopping = True
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
        print('\n현재 세션에 중단 요청 전달; 정지 확인 후 전체 비교를 종료합니다.', flush=True)

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.signal(sig, interrupt)
    try:
        for index, job in enumerate(plan['jobs'], 1):
            if stopping:
                break
            verify_plan(plan)
            old_session = progress['completed'].get(job['id'])
            if old_session:
                okay, reason, _ = inspect_session(old_session, job['runs'])
                if not okay:
                    raise ValueError(f"완료된 세션이 유효하지 않음: {job['id']}: {reason}")
                continue
            session = Path(job['session_dir'])
            if session.exists():
                retry = 2
                while session.with_name(session.name+f'_retry{retry:02d}').exists():
                    retry += 1
                session = session.with_name(session.name+f'_retry{retry:02d}')
            progress.update(state='running', active=job['id'], active_session=str(session))
            save_json(destination/'progress.json', progress)
            print(f"[{index}/{len(plan['jobs'])}] {job['label']} / {job['scenario']} {job['runs']}회", flush=True)
            command = [sys.executable, str(PACKAGE/'scripts/repeat_drive_session.py'),
                       '--scenario', job['scenario'], '--runs', str(job['runs']), '--guarded',
                       '--config-file', job['config_file'], '--session-dir', str(session)]
            child = subprocess.Popen(command, start_new_session=True)
            code = child.wait()
            child = None
            okay, reason, _ = inspect_session(session, job['runs'])
            if stopping or code or not okay:
                progress.update(state='interrupted' if stopping else 'blocked',
                                reason='사용자 중단' if stopping else (reason or f'세션 종료 코드 {code}'))
                save_json(destination/'progress.json', progress)
                print(f"전체 비교 중지: {progress['reason']} / 재개: --resume {destination}", flush=True)
                return 130 if stopping else 1
            progress['completed'][job['id']] = str(session)
            progress.update(active=None, state='running')
            save_json(destination/'progress.json', progress)
            collect_summary(destination, plan, progress)
        progress.update(state='complete' if not stopping else 'interrupted', active=None)
        save_json(destination/'progress.json', progress)
        collect_summary(destination, plan, progress)
        print(f"비교 {'완료' if not stopping else '중단'}: {destination}", flush=True)
        return 130 if stopping else 0
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Prepare or run frozen controller/planner comparisons')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true', help='Prepare and execute the complete sweep')
    mode.add_argument('--resume', type=Path, help='Resume a prepared sweep; completed sets are skipped')
    mode.add_argument('--list', action='store_true', help='List available sets without ROS')
    mode.add_argument('--prepare', action='store_true', help='Prepare configurations only (default)')
    parser.add_argument('--scenarios', nargs='+', default=['scenario1'], choices=SCENARIOS)
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--sets', nargs='+', help='Select catalog sets; default is the9 core sets')
    parser.add_argument('--output', type=Path, help='New sweep directory')
    args = parser.parse_args(argv)
    if args.resume and (args.output or args.sets or args.scenarios != ['scenario1'] or args.runs != 3):
        parser.error('--resume uses the saved scenarios/runs/sets; omit other selection options')
    try:
        if args.list:
            catalog = yaml.safe_load(CATALOG.read_text())
            for name, spec in catalog['sets'].items():
                print(f"{'기본' if name in catalog['default_sets'] else '추가'} {name}: {spec['label']}")
            return 0
        if args.resume:
            return execute(args.resume)
        destination = args.output or WORKSPACE/'parameter_sweeps'/(
            datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:6])
        plan = prepare(destination, args.scenarios, args.runs, args.sets)
        print(f"준비 완료: {len(plan['sets'])}세트 × {len(plan['scenarios'])}시나리오 × {plan['runs']}회 "
              f"= {plan['planned_attempts']}회\n설정·결과: {Path(destination).resolve()}", flush=True)
        return execute(destination) if args.run else 0
    except (ValueError, OSError, KeyError) as error:
        print(f'비교 실행 불가: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
