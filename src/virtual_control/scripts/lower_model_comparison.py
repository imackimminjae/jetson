#!/usr/bin/env python3
"""Run the C6 two-/three-state comparison with the validated driving profile."""
import argparse
import copy
import csv
from datetime import datetime
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo

import yaml

from parameter_sweep import prepare, digest, execute, save_json, validate_config, verify_plan
from repeat_drive_session import WORKSPACE, effective_config

BASELINE = WORKSPACE/'src/virtual_control/config/scenario_bias_baseline.yaml'
RUNTIME = BASELINE.with_name('scenario_bias_runtime.json')
MODELS = {'state2_no_tau': False, 'state3_tau074': True}


def prepare_comparison(destination):
    base = yaml.safe_load(BASELINE.read_text())
    runtime = json.loads(RUNTIME.read_text())
    for binary, key in [('upper_planner_node', 'new_upper_sha256'),
                        ('lower_tracking_mpc_node', 'lower_sha256')]:
        if digest(WORKSPACE/'install/virtual_control/lib/virtual_control'/binary) != runtime[key]:
            raise ValueError(f'{binary}: validated baseline binary changed')
    lower = base['lower_tracking_mpc_node']['ros__parameters']
    if (lower['lower_enable_steering_actuator_model'] is not True or
            lower['lower_steering_actuator_time_constant_sec'] != .74 or
            lower['lower_steering_actuator_delay_sec'] != 0.):
        raise ValueError('Expected the established three-state tau=0.74s, delay=0 baseline')
    validate_config(base)
    # Empty set changes let the existing supervisor prepare ownership, routes,
    # recorder, dependency hashes and resume state. Model changes are frozen below.
    import tempfile
    catalog = dict(version=1, baseline={'knu_batch_runner': {
        'stationary_confirm_sec': .5, 'map_settle_sec': .3, 'reference_min_wait_sec': .3}},
        default_sets=list(MODELS), sets={
        name: dict(label='2-state / no tau' if not enabled else '3-state / tau=0.74s',
                   changes={}) for name, enabled in MODELS.items()})
    with tempfile.TemporaryDirectory(prefix='c6_catalog_') as temporary:
        catalog_file = Path(temporary)/'catalog.yaml'
        catalog_file.write_text(yaml.safe_dump(catalog))
        plan = prepare(destination, ['scenario1', 'scenario3'], 2,
                       catalog_file=catalog_file)
    (destination/'base_effective_config.yaml').write_text(yaml.safe_dump(base, sort_keys=False))
    for job in plan['jobs']:
        config = copy.deepcopy(base)
        enabled = MODELS[job['set']]
        config['lower_tracking_mpc_node']['ros__parameters']['lower_enable_steering_actuator_model'] = enabled
        frozen = Path(job['config_file'])
        frozen.write_text(yaml.safe_dump(config, sort_keys=False))
        config = effective_config(2, Path(job['session_dir'])/'results', job['scenario'],
                                  guarded=True, config_file=frozen)
        validate_config(config)
        frozen.write_text(yaml.safe_dump(config, sort_keys=False))
        u, l = (config[node]['ros__parameters'] for node in
                ('upper_planner_node', 'lower_tracking_mpc_node'))
        job.update(config_sha256=digest(frozen),
                   planner_hz=u['upper_planner_rate_hz'], controller_hz=l['lower_controller_rate_hz'],
                   controller_preview_sec=l['lower_prediction_dt_sec']*l['lower_prediction_steps'],
                   upper_heading_cost=u['lambda_relative_turn'],
                   upper_heading_correction_cost=u['upper_r_dpsi'],
                   upper_steering_limit_rad=u['max_steering_angle_rad'],
                   lower_heading_cost=l['lower_q_heading'],
                   lower_steering_change_cost=l['lower_rd_steering_rate'],
                   lower_steering_rate_radps=l['lower_max_steering_rate_radps'],
                   lower_reset_on_path_change=l['reset_on_path_change'],
                   state_count=3 if enabled else 2, actuator_model_enabled=enabled,
                   effective_tau_sec=.74 if enabled else None,
                   reference_point_to_rear_axle_m=l['lower_reference_point_to_rear_axle_m'])
    plan.update(comparison='C6: remove steering lag state only', baseline_source=str(BASELINE),
                model_difference='lower_enable_steering_actuator_model only; tau ignored when false',
                execution_order=list(MODELS))
    for source in (BASELINE, RUNTIME, Path(__file__), WORKSPACE/'run_c6_mpc_comparison.sh'):
        plan['dependencies_sha256'][str(source.resolve())] = digest(source)
    save_json(destination/'plan.json', plan)
    with (destination/'sets.csv').open('w', newline='') as stream:
        fields = [key for key in plan['jobs'][0] if key not in
                  ('config_file', 'config_sha256', 'session_dir')]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: job[key] for key in fields} for job in plan['jobs'])
    verify_plan(plan)
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.resume and (args.output or args.prepare_only):
        parser.error('--resume cannot be combined with preparation options')
    if args.resume:
        return execute(args.resume)
    stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    destination = (args.output or WORKSPACE/'parameter_sweeps'/f'c6_state2_vs_state3_{stamp}').resolve()
    prepare_comparison(destination)
    print('2-state(no tau) vs 3-state(tau=0.74s): scenarios 1,3 x 2 each = 8 attempts', flush=True)
    print(f'Frozen configurations and results: {destination}', flush=True)
    return 0 if args.prepare_only else execute(destination)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError) as error:
        print(f'Comparison unavailable: {error}', file=sys.stderr)
        raise SystemExit(1)
