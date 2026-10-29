#!/usr/bin/env python3
"""Freeze and run the C5 hybrid-vs-all-BnB comparison with identical tuning."""
import argparse
import copy
import csv
from datetime import datetime
import json
from pathlib import Path
import tempfile
from zoneinfo import ZoneInfo
import yaml
from parameter_sweep import prepare, digest, execute, save_json, validate_config, verify_plan
from repeat_drive_session import WORKSPACE, effective_config


def prepare_comparison(destination):
    baseline=WORKSPACE/'src/virtual_control/config/scenario_bias_baseline.yaml'
    base=yaml.safe_load(baseline.read_text());validate_config(base)
    variants={'hybrid':False,'bnb':True}
    catalog=dict(version=1,baseline={'knu_batch_runner':{'stationary_confirm_sec':.5,
        'map_settle_sec':.3,'reference_min_wait_sec':.3}},default_sets=list(variants),sets={
        mode:dict(label='All BnB' if bnb else 'Legacy hybrid',changes={}) for mode,bnb in variants.items()})
    with tempfile.TemporaryDirectory(prefix='c5_catalog_') as temporary:
        path=Path(temporary)/'catalog.yaml';path.write_text(yaml.safe_dump(catalog))
        plan=prepare(destination,['scenario1','scenario3'],2,catalog_file=path)
    (destination/'base_effective_config.yaml').write_text(yaml.safe_dump(base,sort_keys=False))
    for job in plan['jobs']:
        config=copy.deepcopy(base)
        config['upper_planner_node']['ros__parameters']['upper_miqp_force_bnb']=variants[job['set']]
        frozen=Path(job['config_file']);frozen.write_text(yaml.safe_dump(config,sort_keys=False))
        config=effective_config(2,Path(job['session_dir'])/'results',job['scenario'],guarded=True,config_file=frozen)
        validate_config(config);frozen.write_text(yaml.safe_dump(config,sort_keys=False))
        u,l=(config[node]['ros__parameters'] for node in ('upper_planner_node','lower_tracking_mpc_node'))
        job.update(config_sha256=digest(frozen),force_bnb=variants[job['set']],
            planner_hz=u['upper_planner_rate_hz'],controller_hz=l['lower_controller_rate_hz'],
            controller_preview_sec=l['lower_prediction_dt_sec']*l['lower_prediction_steps'],
            upper_heading_cost=u['lambda_relative_turn'],upper_heading_correction_cost=u['upper_r_dpsi'],
            upper_steering_limit_rad=u['max_steering_angle_rad'],lower_heading_cost=l['lower_q_heading'],
            lower_steering_change_cost=l['lower_rd_steering_rate'],lower_steering_rate_radps=l['lower_max_steering_rate_radps'],
            lower_reset_on_path_change=l['reset_on_path_change'],state_count=3,
            actuator_model_enabled=l['lower_enable_steering_actuator_model'],effective_tau_sec=.74)
    plan.update(comparison='C5: original hybrid vs BnB for every upper MIQP',
        only_parameter_difference='upper_miqp_force_bnb',baseline_source=str(baseline))
    for source in (baseline,Path(__file__),WORKSPACE/'run_c5_bnb_comparison.sh',
                   WORKSPACE/'src/virtual_control/src/tracking_control.cpp',
                   WORKSPACE/'src/virtual_control/include/virtual_control/tracking_control.hpp'):
        plan['dependencies_sha256'][str(source.resolve())]=digest(source)
    save_json(destination/'plan.json',plan);verify_plan(plan)
    fields=[k for k in plan['jobs'][0] if k not in ('config_file','config_sha256','session_dir')]
    with (destination/'sets.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(plan['jobs'])
    print(f'Hybrid vs all-BnB: scenarios 1,3 x 2 each = 8 attempts\n{destination}',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only',action='store_true');parser.add_argument('--resume',type=Path)
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    if args.resume:
        if args.output or args.prepare_only:parser.error('--resume uses the frozen plan')
        return execute(args.resume)
    stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')
    destination=(args.output or WORKSPACE/'parameter_sweeps'/f'c5_hybrid_vs_bnb_{stamp}').resolve()
    prepare_comparison(destination)
    return 0 if args.prepare_only else execute(destination)


if __name__=='__main__':
    raise SystemExit(main())
