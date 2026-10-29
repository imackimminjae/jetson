#!/usr/bin/env python3
"""Aggregate driving outcomes, actual solver times and solver-policy audit."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from analyze_lower_model_comparison import analyze as analyze_tracking


def analyze(root):
    analyze_tracking(root)
    plan=json.loads((root/'plan.json').read_text());progress=json.loads((root/'progress.json').read_text())
    results=[]
    for job in plan['jobs']:
        if job['id'] not in progress['completed']:continue
        session=Path(progress['completed'][job['id']])
        with next(session.glob('results/*/summary.csv')).open() as stream:attempts=list(csv.DictReader(stream))
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=['/knu_batch/status','/debug/upper_branch_event']))
        phases={};events=[]
        while reader.has_next():
            topic,raw,_=reader.read_next();item=json.loads(deserialize_message(raw,String).data)
            if topic=='/knu_batch/status':phases[item['sequence']]=item
            else:events.append(item)
        del reader
        phases=sorted(phases.values(),key=lambda s:s['phase_stamp_sec'])
        for number,attempt in enumerate(attempts,1):
            start=next(s['phase_stamp_sec'] for s in phases if s.get('token')==attempt['token'] and s['phase']=='running')
            end=next(s['phase_stamp_sec'] for s in phases if s['phase_stamp_sec']>start and s['phase']!='running')
            active=[e for e in events if start<=e['t_start']<end]
            called=[e for e in active if e.get('solver_called')]
            if not called:raise ValueError(f'{session}: no solver calls')
            for e in active:
                assert e['miqp_force_bnb'] is job['force_bnb']
                if job['force_bnb']:assert e['enumerated_corridor_combinations']==0
            times=np.array([e['solver_time_ms'] for e in called])
            cycles=np.array([(e['t_emit']-e['t_start'])*1000 for e in active])
            row=dict(mode=job['set'],scenario=job['scenario'],attempt=number,outcome=attempt['outcome'],
                planner_cycles=len(active),solver_cycles=len(called),
                solver_rejections=sum(not e['solver_valid'] for e in called),
                bnb_numerical_failures=sum(e.get('status','').startswith('DAQP branch-and-bound MIQP failed') for e in called),
                feasibility_rejections=sum(e.get('status','').startswith('DAQP MIQP rejected') for e in called),
                reference_road_rejections=sum(e.get('reason')=='hard_road_rejected' for e in active),
                published_plans=sum(e['valid_plan'] for e in active),
                retained_path_cycles=sum(e['retained_previous_path'] for e in active),
                solver_sum_mean_ms=float(times.mean()),solver_sum_p95_ms=float(np.percentile(times,95)),
                solver_sum_max_ms=float(times.max()),cycle_mean_ms=float(cycles.mean()),
                cycle_p95_ms=float(np.percentile(cycles,95)),cycle_gt100_pct=float(100*np.mean(cycles>100)),
                bnb_calls=sum(e['bnb_solve_calls'] for e in active),
                enumerated_combinations=sum(e['enumerated_corridor_combinations'] for e in active),
                selected_corridor_polishes=sum(e['selected_corridor_polish_calls'] for e in active))
            results.append(row)
    if not results:return
    with (root/'upper_solver_metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    groups=[]
    for mode in plan['sets']:
        for scenario in plan['scenarios']:
            rows=[r for r in results if r['mode']==mode and r['scenario']==scenario]
            if not rows:continue
            group=dict(mode=mode,scenario=scenario,runs=len(rows),completed=sum(r['outcome']=='completed' for r in rows))
            for key in ('planner_cycles','solver_cycles','solver_rejections','published_plans','retained_path_cycles',
                        'bnb_numerical_failures','feasibility_rejections','reference_road_rejections',
                        'bnb_calls','enumerated_combinations','selected_corridor_polishes'):
                group[key]=sum(r[key] for r in rows)
            for key in ('solver_sum_mean_ms','solver_sum_p95_ms','solver_sum_max_ms','cycle_mean_ms','cycle_p95_ms','cycle_gt100_pct'):
                group[f'mean_{key}']=float(np.mean([r[key] for r in rows]))
            groups.append(group)
    with (root/'upper_solver_groups.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(groups[0]));writer.writeheader();writer.writerows(groups)
    report=dict(state=progress['state'],attempts=len(results),groups=groups,
        timing='Sum of measured upper MIQP/BnB, incumbent-polish QP and physical-refinement QP calls; cycle time also includes input processing and geometry.',
        policy='Every BnB integer choice is made by DAQP BnB. Continuous polishing fixes that selection and never enumerates other selections.')
    (root/'solver_comparison_result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    analyze(parser.parse_args().directory.resolve())
