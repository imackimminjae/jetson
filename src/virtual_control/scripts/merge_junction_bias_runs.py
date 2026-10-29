#!/usr/bin/env python3
"""Combine verified driving segments after a recorder-only interruption.

Original recordings and frozen plans stay intact. Logical repetition numbers
refer to the requested three drives; source session/run remain explicit.
"""
import argparse
from collections import Counter, defaultdict
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def read(path):
    return json.loads(Path(path).read_text())


def save(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--allow-partial', action='store_true')
    args = parser.parse_args()
    batch = args.batch.resolve()
    registry = read(batch/'continuations.json')
    plans = [Path(registry['original_plan']), *map(Path, registry['continuation_plans'])]
    destination = batch/'final/analysis'
    destination.mkdir(parents=True, exist_ok=True)
    records = []

    def append(source_row, source_npz, source_plan, source_json, offset):
        row = copy.deepcopy(source_row)
        source_npz = source_npz.resolve()
        assert source_npz.is_file()
        row.update(run=source_row['run']+offset, source_run=source_row['run'],
                   source_plan=str(source_plan.relative_to(batch)),
                   source_audit=str(source_json.relative_to(batch)),
                   source_npz=str(source_npz.relative_to(batch)))
        row['lower_cycles_without_valid_qp_or_fallback'] = row.pop('lower_failed_cycles')
        link = destination/f'{row["job"]}_run{row["run"]}.npz'
        if link.exists() or link.is_symlink():
            assert link.resolve()==source_npz, (link, source_npz)
        else:
            link.symlink_to(os.path.relpath(source_npz, link.parent))
        records.append(row)

    for path in plans:
        subprocess.run([sys.executable, str(Path(__file__).with_name('analyze_junction_bias_runs.py')),
                        str(path.parent)], check=True)
        plan, progress = read(path), read(path.parent/'progress.json')
        jobs = {j['id']: j for j in plan['jobs']}
        offsets = registry.get('logical_run_offsets', {}).get(str(path), {})
        for job_id, session in progress['completed'].items():
            filename = path.parent/'analysis'/f'{job_id}.json'
            # The driver can finish another session after the audit subprocess
            # took its progress snapshot. Audit that newly finished bag next time.
            if not filename.exists():
                continue
            cache = read(filename)
            assert cache['session']==session and len(cache['attempts'])==jobs[job_id]['runs']
            for row in cache['attempts']:
                source_npz = filename.parent/f'{job_id}_run{row["run"]}.npz'
                append(row, source_npz, path, filename, offsets.get(job_id, 0))
    for segment in registry['preserved_partial_measurements']:
        filename = Path(segment['audit_file'])
        cache = read(filename)
        assert len(cache['attempts'])==segment['valid_runs']
        for row in cache['attempts']:
            source_npz = filename.parent/f'{row["job"]}_run{row["run"]}.npz'
            append(row, source_npz, Path(segment['source_plan']), filename, segment['logical_run_offset'])

    records.sort(key=lambda r: (r['bias_deg'], int(r['scenario'].removeprefix('scenario')), r['run']))
    identities = [(r['scenario'], r['bias_deg'], r['run']) for r in records]
    assert len(set(identities))==len(identities), 'duplicate measured repetitions'
    counts = Counter((r['scenario'], r['bias_deg']) for r in records)
    assert all(0<count<=3 for count in counts.values()) and len(records)<=90
    assert all(r['stop_status']=='stationary' for r in records)
    latest = read(plans[-1].parent/'progress.json')
    complete = latest['state']=='complete' and len(records)==90
    if complete:
        assert set(counts)=={(f'scenario{n}', a) for n in range(26, 32) for a in [-10, -5, 0, 5, 10]}
        assert all(count==3 for count in counts.values())
    elif not args.allow_partial:
        raise ValueError(f'Final dataset not ready: {len(records)}/90 audited driving attempts')
    grouped = defaultdict(list)
    for row in records:
        grouped[row['job']].append(row)
    for job_id, attempts in grouped.items():
        save(destination/f'{job_id}.json', dict(attempts=attempts,
            provenance='Real recordings; source session and source run are retained in every row'))
    if records:
        with (destination/'attempt_metrics.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader(); writer.writerows(records)
    report = dict(state='complete' if complete else latest['state'], complete=complete,
        planned_attempts=90, audited_attempts=len(records), conditions=len(counts),
        repetitions_per_condition=3, outcomes=dict(Counter(r['outcome'] for r in records)),
        all_stops_stationary=True, actual_window_verified=bool(records),
        baseline='3-state tau .74s + DAQP BnB; original road and goal',
        bnb_calls=sum(r['bnb_calls'] for r in records), enumeration_calls=0,
        upper_failed_calls=sum(r['upper_failed_calls'] for r in records),
        input_timeout_attempts=[dict(scenario=r['scenario'], bias_deg=r['bias_deg'],
            run=r['run'], outcome=r['outcome'], source_session=r['session'])
            for r in records if r['outcome']=='pose_timeout'],
        full_pose_coverage_for_all_attempts=not any(r['outcome']=='pose_timeout' for r in records),
        excluded_setup_aborts=registry.get('excluded_setup_aborts', []),
        plans=[dict(path=str(p.relative_to(batch)), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in plans],
        repetition_policy='Exactly three actual driving attempts per scenario and bias. '
                          'The stopped, pre-driving recorder abort is excluded. '
                          'Driving failures such as off_route and pose_timeout remain in the three. '
                          'Position gaps are not interpolated.')
    save(destination/'audit_summary.json', report)
    print(json.dumps(report, ensure_ascii=False))


if __name__=='__main__':
    main()
