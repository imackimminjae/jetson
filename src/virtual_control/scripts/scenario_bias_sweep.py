"""Freeze and run selected scenarios and waypoint yaw biases."""
import argparse
import copy
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import uuid
from zoneinfo import ZoneInfo

import yaml

W = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(W / 'src/virtual_control/scripts'))
from parameter_sweep import prepare, validate_config, verify_plan, digest, save_json, execute
from repeat_drive_session import AVAILABLE_SCENARIOS as SCENARIOS, effective_config
from knu_global_path_publisher import load_route
from waypoint_bias_window import parse_bias_windows


def main(argv=None):
    parser = argparse.ArgumentParser(description='선택한 시나리오와 yaw 바이어스를 순차 실행')
    parser.add_argument('scenario', type=int, nargs='+', choices=[int(s.removeprefix('scenario')) for s in SCENARIOS])
    parser.add_argument('--bias', type=int, nargs='+', choices=[-10, -5, 0, 5, 10],
                        default=[-10, -5, 0, 5, 10], help='yaw 바이어스 각도 (도 단위, 기본: 5종 모두)')
    parser.add_argument('--runs', '--runs-per-bias', dest='runs_per_bias', type=int, default=1,
                        help='시나리오·바이어스 조합별 반복 수 (기본 1)')
    parser.add_argument('--order', choices=['bias', 'scenario'], default='bias',
                        help='실행 묶음 순서: bias는 한 바이어스에서 선택 시나리오 전체 실행 (기본), scenario는 시나리오별 실행')
    parser.add_argument('--prepare-only', action='store_true', help='설정 저장만 하고 주행하지 않음')
    parser.add_argument('--soft-ratio', type=float, default=None,
                        help='중심 수축 비율 (0.5 초과, 1.0 이하; 생략 시 기준 설정 사용)')
    parser.add_argument('--route-prefix', action='append', default=[], metavar='SCENARIO:COUNT',
                        help='선택한 시나리오의 앞 COUNT개 웨이포인트만 사용 (예: 11:6)')
    parser.add_argument('--bias-waypoints', action='append', default=[], metavar='[SCENARIO:]START:END',
                        help='현재 wp0가 지정한 1기준 번호 범위에 있을 때만 차량 중심 방향 바이어스 적용. '
                             'START:END는 전체 선택 시나리오 공통, SCENARIO:START:END는 개별 범위')
    args = parser.parse_args(argv)
    if args.soft_ratio is not None and not (math.isfinite(args.soft_ratio) and .5 < args.soft_ratio <= 1.):
        parser.error('--soft-ratio는 0.5 초과, 1.0 이하의 유한한 값이어야 합니다')
    if not 1 <= args.runs_per_bias <= 100:
        parser.error('--runs-per-bias는 1~100이어야 합니다')
    if len(set(args.scenario)) != len(args.scenario) or len(set(args.bias)) != len(args.bias):
        parser.error('시나리오와 바이어스는 중복 없이 지정하세요')
    scenarios = [f'scenario{number}' for number in args.scenario]
    prefixes = {}
    for specification in args.route_prefix:
        try:
            number, count = map(int, specification.split(':'))
            name = f'scenario{number}'
            if name not in scenarios or count < 2:
                raise ValueError('선택한 시나리오와 2 이상의 웨이포인트 수를 지정하세요')
            load_route(name, W/'src/virtual_control/data/knu_routes', count)
            prefixes[name] = count
        except ValueError as error:
            parser.error(f'--route-prefix {specification}: {error}')
    runs = args.runs_per_bias
    try:
        windows = parse_bias_windows(args.bias_waypoints, scenarios,
                                     W/'src/virtual_control/data/knu_routes', prefixes)
    except ValueError as error:
        parser.error(str(error))
    angles = args.bias
    bias_names = {-10: 'bias_m10', -5: 'bias_m05', 0: 'bias_000', 5: 'bias_p05', 10: 'bias_p10'}
    names = [bias_names[angle] for angle in angles]
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    destination = W / 'parameter_sweeps' / f'manual_bias_{"_".join(scenarios)}_{stamp}_{uuid.uuid4().hex[:6]}'
    basefile = W / 'src/virtual_control/config/scenario_bias_baseline.yaml'
    base = yaml.safe_load(basefile.read_text())
    upper = base['upper_planner_node']['ros__parameters']
    if args.soft_ratio is not None:
        upper['preview_interval_soft_ratio'] = args.soft_ratio
    if not upper['upper_soft_corridor_constraints'] or upper['skip_front_clipped_k1_lane_constraints'] or upper['preview_interval_k1_disable_extra_centering']:
        raise ValueError('도로 hard / 선호 구간 soft / k1 예외 없음 설정을 확인하세요')
    runtimefile = basefile.parent / 'scenario_bias_runtime.json'
    install = json.loads(runtimefile.read_text())
    for binary, expected in [('upper_planner_node', install['new_upper_sha256']), ('lower_tracking_mpc_node', install['lower_sha256'])]:
        if digest(W / 'install/virtual_control/lib/virtual_control' / binary) != expected:
            raise ValueError(f'{binary} 실행 파일이 기존 동결 버전과 다릅니다')
    window_runtime = None
    if windows:
        window_runtime = basefile.parent/'scenario_waypoint_bias_runtime.json'
        window_install = json.loads(window_runtime.read_text())
        window_binary = W/'install/virtual_control/lib/virtual_control/upper_planner_waypoint_bias_node'
        if digest(window_binary) != window_install['upper_sha256']:
            raise ValueError('부분 바이어스 실험 실행 파일이 준비된 버전과 다릅니다')
    # Reuse the original frozen catalog and unchanged guarded session runner.
    catalog = W / 'src/virtual_control/config/scenario_bias_catalog.yaml'
    plan = prepare(destination, scenarios, runs, selected=names, catalog_file=catalog)
    # Complete all selected scenarios at one bias before changing the bias.
    if args.order == 'bias':
        plan['jobs'].sort(key=lambda job: (names.index(job['set']), scenarios.index(job['scenario'])))
    else:
        plan['jobs'].sort(key=lambda job: (scenarios.index(job['scenario']), names.index(job['set'])))
    (destination / 'base_effective_config.yaml').write_text(yaml.safe_dump(base, sort_keys=False))
    for job in plan['jobs']:
        angle = angles[names.index(job['set'])]
        config = copy.deepcopy(base)
        if job['scenario'] in prefixes:
            config['knu_batch_runner']['ros__parameters']['route_end_waypoint'] = prefixes[job['scenario']]
        config['upper_planner_node']['ros__parameters'].update(enable_waypoint_bias=True, waypoint_yaw_bias_rad=math.radians(angle))
        if windows:
            start, end = windows[job['scenario']]
            config['upper_planner_node']['ros__parameters'].update(
                waypoint_bias_mode='waypoint_window', waypoint_bias_start_wp=start,
                waypoint_bias_end_wp=end)
        frozen = Path(job['config_file'])
        frozen.write_text(yaml.safe_dump(config, sort_keys=False))
        config = effective_config(runs, Path(job['session_dir']) / 'results', job['scenario'], guarded=True, config_file=frozen)
        validate_config(config)
        frozen.write_text(yaml.safe_dump(config, sort_keys=False))
        u, l = (config[node]['ros__parameters'] for node in ('upper_planner_node', 'lower_tracking_mpc_node'))
        job.update(config_sha256=digest(frozen), waypoint_bias_deg=angle, waypoint_bias_rad=math.radians(angle),
                   planner_hz=u['upper_planner_rate_hz'], controller_hz=l['lower_controller_rate_hz'],
                   controller_preview_sec=l['lower_prediction_dt_sec'] * l['lower_prediction_steps'],
                   upper_heading_cost=u['lambda_relative_turn'], upper_heading_correction_cost=u['upper_r_dpsi'],
                   upper_steering_change_cost=u.get('upper_r_steering_delta', 0.0),
                   upper_steering_limit_rad=u['max_steering_angle_rad'], lower_heading_cost=l['lower_q_heading'],
                   lower_steering_change_cost=l['lower_rd_steering_rate'], lower_steering_rate_radps=l['lower_max_steering_rate_radps'],
                   lower_reset_on_path_change=l['reset_on_path_change'],
                   preview_interval_soft_ratio=u['preview_interval_soft_ratio'],
                   route_end_waypoint=prefixes.get(job['scenario'], 0))
        if windows:
            job.update(bias_waypoints=windows[job['scenario']], bias_mode='waypoint_window')
    plan.update(baseline_source=str(basefile), enabled_profile=str(basefile),
                bias_degrees=angles, soft_corridors=True, k1_exceptions=False,
                preview_interval_soft_ratio=upper['preview_interval_soft_ratio'],
                execution_order=args.order,
                route_prefixes=prefixes)
    if windows:
        plan.update(bias_mode='waypoint_window', bias_waypoints=windows,
                    waypoint_numbering='1-based inclusive current wp0; original route selects progress and goal',
                    bias_anchor='measured vehicle position')
        for source in (window_runtime, window_binary):
            plan['dependencies_sha256'][str(source.resolve())] = digest(source)
    for source in (basefile, runtimefile, catalog, Path(__file__),
                   Path(__file__).with_name('waypoint_bias_window.py'), W / 'run_scenario_bias.sh'):
        plan['dependencies_sha256'][str(source)] = digest(source)
    save_json(destination / 'plan.json', plan)
    verify_plan(plan)
    with (destination / 'sets.csv').open('w') as stream:
        fields = [k for k in plan['jobs'][0] if k not in ('config_file', 'config_sha256', 'session_dir')]
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(plan['jobs'])
    print(f'{", ".join(scenarios)}: {", ".join(f"{angle:+d}°" for angle in angles)} × 각 {runs}회 = 총 {plan["planned_attempts"]}회', flush=True)
    print(f'설정·로그·결과: {destination}', flush=True)
    print('도로 hard / 선호 구간 soft / k1 예외 없음 / P10 C20 RD600', flush=True)
    print(f'중심 수축 비율: {upper["preview_interval_soft_ratio"]}', flush=True)
    print(f'상위 조향 변화 비용: {upper.get("upper_r_steering_delta", 0.0)}', flush=True)
    print('실행 순서: '+('바이어스별 선택 시나리오 전체' if args.order == 'bias' else '시나리오별 선택 바이어스 전체'), flush=True)
    if windows:
        print('부분 바이어스: 현재 wp0의 1기준 번호 / '+', '.join(
            f'{name}={start}~{end}번' for name, (start, end) in windows.items()), flush=True)
    return 0 if args.prepare_only else execute(destination)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError) as error:
        print(f'실행 불가: {error}', file=sys.stderr)
        raise SystemExit(1)
