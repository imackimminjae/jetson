#!/usr/bin/env python3
"""Extract driving-only lower-MPC errors from the recorded C6 comparison."""
import argparse
import csv
import json
from pathlib import Path


def analyze(destination):
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray, String

    plan = json.loads((destination/'plan.json').read_text())
    progress = json.loads((destination/'progress.json').read_text())
    rows = []
    for job in plan['jobs']:
        recorded = progress['completed'].get(job['id'])
        if recorded is None:
            continue
        session = Path(recorded)
        summaries = list(session.glob('results/*/summary.csv'))
        if len(summaries) != 1:
            raise ValueError(f'{session}: expected one attempt summary')
        with summaries[0].open() as stream:
            attempts = list(csv.DictReader(stream))
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(
            topics=['/knu_batch/status', '/debug/lower_mpc_trace']))
        phases, traces = {}, []
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == '/knu_batch/status':
                status = json.loads(deserialize_message(raw, String).data)
                phases[status['sequence']] = status
            else:
                trace = deserialize_message(raw, Float64MultiArray).data
                if len(trace) < 32:
                    raise ValueError('Expected the current 32-field lower trace')
                traces.append(list(trace))
        del reader
        traces = np.asarray(traces)
        if traces.size == 0:
            raise ValueError(f'{session}: no lower traces')
        phases = sorted(phases.values(), key=lambda s: s['phase_stamp_sec'])
        for index, attempt in enumerate(attempts, 1):
            running = [s for s in phases if s.get('token') == attempt['token'] and s['phase'] == 'running']
            if not running:
                raise ValueError(f'{session}: attempt {index} has no recorded driving interval')
            start = running[0]['phase_stamp_sec']
            end = next(s['phase_stamp_sec'] for s in phases
                       if s['phase_stamp_sec'] > start and s['phase'] != 'running')
            active = traces[(traces[:, 0] >= start) & (traces[:, 0] < end)]
            if not len(active):
                raise ValueError(f'{session}: attempt {index} has no driving traces')
            enabled = float(job['actuator_model_enabled'])
            if not np.all(active[:, 21] == enabled):
                raise ValueError(f'{session}: recorded model differs from frozen configuration')
            if not np.allclose(active[:, 23], .74) or not np.allclose(active[:, 30], 1.4):
                raise ValueError(f'{session}: tau or vehicle reference point changed')
            valid = active[(active[:, 10] == 1) | (active[:, 27] == 1)]
            if not len(valid) or not np.isfinite(valid[:, [6, 7, 11, 26]]).all():
                raise ValueError(f'{session}: valid tracking diagnostics missing')
            # Neutral/stop outputs have valid=0 without attempting a QP.
            # Count actual measured solver calls, not those transition samples.
            qp_calls = active[np.isfinite(active[:, 24])]
            if not len(qp_calls):
                raise ValueError(f'{session}: no measured QP calls')
            row = dict(model=job['set'], scenario=job['scenario'], attempt=index,
                       outcome=attempt['outcome'], duration_sec=float(attempt['duration_sec']),
                       progress_ratio=float(attempt['progress_ratio']),
                       trace_samples=len(active), valid_error_samples=len(valid),
                       valid_error_coverage_pct=float(100*len(valid)/len(active)),
                       qp_calls=len(qp_calls), qp_failures=int(np.sum(qp_calls[:, 10] != 1)),
                       lateral_rms_m=float(np.sqrt(np.mean(valid[:, 6]**2))),
                       lateral_p95_m=float(np.percentile(abs(valid[:, 6]), 95)),
                       lateral_max_m=float(np.max(abs(valid[:, 6]))),
                       body_heading_rms_deg=float(np.degrees(np.sqrt(np.mean(valid[:, 7]**2)))),
                       qp_failure_pct=float(100*np.mean(qp_calls[:, 10] != 1)),
                       fallback_pct=float(100*np.mean(active[:, 12] == 5)),
                       solve_p95_ms=float(np.percentile(valid[:, 11], 95)),
                       steering_max_rad=float(np.max(abs(active[:, 26]))),
                       session=str(session))
            rows.append(row)
    if not rows:
        print('No completed driving sessions; no performance numbers generated.')
        return
    with (destination/'comparison_metrics.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    metrics = ('lateral_rms_m', 'lateral_p95_m', 'lateral_max_m', 'body_heading_rms_deg',
               'valid_error_coverage_pct',
               'qp_failure_pct', 'fallback_pct', 'solve_p95_ms', 'steering_max_rad')
    groups = []
    for model in plan['sets']:
        for scenario in plan['scenarios']:
            selected = [r for r in rows if r['model'] == model and r['scenario'] == scenario]
            if not selected:
                continue
            group = dict(model=model, scenario=scenario, runs=len(selected),
                         completed=sum(r['outcome'] == 'completed' for r in selected))
            group.update({f'mean_{key}': float(np.mean([r[key] for r in selected])) for key in metrics})
            groups.append(group)
    with (destination/'comparison_groups.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(groups[0]))
        writer.writeheader(); writer.writerows(groups)
    print(json.dumps(dict(state=progress['state'], analyzed_attempts=len(rows), groups=groups), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    analyze(parser.parse_args().destination.resolve())
