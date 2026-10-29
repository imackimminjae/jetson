#!/usr/bin/env python3
"""Audit completed short-bias recordings without touching a running session."""
import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray
import yaml


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def audit(job, session, destination, accepted_tokens=None):
    config = yaml.safe_load((session/'effective_config.yaml').read_text())
    upper = config['upper_planner_node']['ros__parameters']
    lower = config['lower_tracking_mpc_node']['ros__parameters']
    assert upper['upper_miqp_force_bnb'] is True
    assert lower['lower_enable_steering_actuator_model'] is True
    assert lower['lower_steering_actuator_time_constant_sec'] == .74
    assert upper['waypoint_bias_mode'] == 'waypoint_window'
    assert lower['odom_yaw_is_orientation_z'] is True
    angle = job['waypoint_bias_rad']
    assert math.isclose(angle, upper['waypoint_yaw_bias_rad'], abs_tol=1e-12)
    summary, = session.glob('results/*/summary.csv')
    attempts = list(csv.DictReader(summary.open()))
    if accepted_tokens is not None:
        attempts = [r for r in attempts if r['token'] in accepted_tokens]
    assert len(attempts) == job['runs']
    route = np.loadtxt(session/'routes'/f'{job["scenario"]}.csv', delimiter=',', skiprows=1)[:, :2]
    spawn = next(m for m in json.loads((session/'routes/knu_start_poses.json').read_text())['starts']
                 if m['scenario'] == job['scenario'])
    phases, poses, events, traces = {}, [], [], []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[
        '/knu_batch/status', '/motive/vehicle/odom_map', '/debug/upper_branch_event',
        '/debug/lower_mpc_trace']))
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        if topic == '/knu_batch/status':
            message = json.loads(deserialize_message(raw, String).data)
            phases[message['sequence']] = message
        elif topic == '/motive/vehicle/odom_map':
            message = deserialize_message(raw, Odometry)
            t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            poses.append([t or stamp * 1e-9, message.pose.pose.position.x,
                          message.pose.pose.position.y, message.pose.pose.orientation.z])
        elif topic == '/debug/upper_branch_event':
            events.append(json.loads(deserialize_message(raw, String).data))
        else:
            traces.append(list(deserialize_message(raw, Float64MultiArray).data))
    del reader
    phases = sorted(phases.values(), key=lambda r: r['phase_stamp_sec'])
    poses = np.asarray(poses)
    traces = np.asarray(traces)
    output = []
    for run, attempt in enumerate(attempts, 1):
        start = next(p['phase_stamp_sec'] for p in phases
                     if p.get('token') == attempt['token'] and p['phase'] == 'running')
        end = next(p['phase_stamp_sec'] for p in phases
                   if p['phase_stamp_sec'] > start and p['phase'] != 'running')
        selected = [e for e in events if start <= e['t_start'] < end]
        assert selected, (job['id'], run, 'missing upper events')
        assert all(e['miqp_force_bnb'] is True for e in selected)
        assert all(e['enumerated_corridor_combinations'] == 0 for e in selected)
        assert all(e['waypoint_bias_mode'] == 'waypoint_window' for e in selected)
        assert all(math.isclose(e['waypoint_bias_requested_rad'], angle, abs_tol=1e-7) for e in selected)
        mask = np.array([e['waypoint_bias_active'] for e in selected], dtype=bool)
        sequence = [bool(mask[0])]
        sequence += [bool(v) for v in mask[1:][mask[1:] != mask[:-1]]]
        proof_errors = []
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        for e in selected:
            if not e['valid_plan']:
                continue
            position = np.array([e['x'], e['y']])
            target = route[e['wp1']]
            expected = position + rotation @ (target-position) if e['waypoint_bias_active'] else target
            proof_errors.append(float(np.linalg.norm(np.asarray(e['planning_wp1_world']) - expected)))
        assert proof_errors and max(proof_errors) < .002, (job['id'], run, max(proof_errors, default=-1))
        if attempt['outcome'] == 'completed':
            assert sequence == [False, True, False], (job['id'], run, sequence)
        active = [e for e in selected if e['waypoint_bias_active']]
        pose = poses[(poses[:, 0] >= start) & (poses[:, 0] < end)].copy()
        assert len(pose) > 10
        pose[:, 0] -= start
        pose[:, 3] = np.unwrap(pose[:, 3])
        pose[:, 3] += 2*np.pi*np.round((spawn['heading_rad'] - pose[0, 3])/(2*np.pi))
        trace = traces[(traces[:, 0] >= start) & (traces[:, 0] < end)]
        assert len(trace) > 10 and np.all(trace[:, 21] == 1.) and np.allclose(trace[:, 23], .74)
        record = dict(job=job['id'], scenario=job['scenario'], bias_deg=job['waypoint_bias_deg'],
            run=run, outcome=attempt['outcome'], duration_sec=float(attempt['duration_sec']),
            stop_status=attempt['stop_status'], pose_samples=len(pose),
            bias_state_sequence=sequence, bias_cycles=int(mask.sum()),
            bias_on_sec=float(active[0]['t_start']-start) if active else None,
            bias_off_sec=float(next((e['t_start']-start for e in selected
                if active and e['t_start'] > active[-1]['t_start'] and not e['waypoint_bias_active']), end-start)) if active else None,
            bias_on_xy=[active[0]['x'], active[0]['y']] if active else None,
            bias_last_active_xy=[active[-1]['x'], active[-1]['y']] if active else None,
            planning_waypoint_rotation_max_error_m=max(proof_errors),
            bnb_calls=sum(e['bnb_solve_calls'] for e in selected), enumeration_calls=0,
            upper_failed_calls=sum(e['solver_called'] and not e['solver_valid'] for e in selected),
            lower_failed_cycles=int(np.sum((trace[:, 10] != 1.) & (trace[:, 27] != 1.))),
            heading_source='map-aligned EKF body yaw, scalar orientation.z',
            session=str(session))
        np.savez_compressed(destination/f'{job["id"]}_run{run}.npz', pose_time_xy_yaw=pose,
            route_xy=route, upper_event_t_active_wp0=np.array([
                [e['t_start']-start, int(e['waypoint_bias_active']), e['wp0']] for e in selected]))
        output.append(record)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    args = parser.parse_args()
    batch = args.batch.resolve()
    plan = json.loads((batch/'plan.json').read_text())
    progress = json.loads((batch/'progress.json').read_text())
    destination = batch/'analysis'
    destination.mkdir(exist_ok=True)
    records = []
    for job in plan['jobs']:
        session = progress['completed'].get(job['id'])
        if not session:
            continue
        filename = destination/f'{job["id"]}.json'
        if filename.exists():
            result = json.loads(filename.read_text())
            assert result['session'] == session
            rows = result['attempts']
        else:
            rows = audit(job, Path(session), destination)
            save_json(filename, dict(session=session, attempts=rows))
        records.extend(rows)
    report = dict(state=progress['state'], planned_attempts=plan['planned_attempts'],
        audited_attempts=len(records), completed_sessions=len(progress['completed']),
        outcomes=dict(Counter(r['outcome'] for r in records)),
        all_stops_stationary=all(r['stop_status']=='stationary' for r in records),
        baseline='3-state tau .74s + DAQP BnB; original road and goal',
        bnb_calls=sum(r['bnb_calls'] for r in records), enumeration_calls=0,
        upper_failed_calls=sum(r['upper_failed_calls'] for r in records),
        actual_window_verified=True, complete=progress['state']=='complete' and len(records)==plan['planned_attempts'])
    save_json(destination/'audit_summary.json', report)
    if records:
        with (destination/'attempt_metrics.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(records[0]))
            writer.writeheader(); writer.writerows(records)
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
