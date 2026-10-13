"""Review latest local batches, counting grid records without decoding them."""
from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import csv
import gzip
import hashlib
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s7812_review_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s7812_review_mpl'
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCHES = {3: '20261013_024858_scenario3_5c4c45'}
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 9})

def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

def hash_file(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()

def max_gap(values):
    return float(np.max(np.diff(values))) if len(values) > 1 else None

def output_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

all_attempts, all_summaries, figures, sources = [], [], [], {}
batch_data = {}
for n, name in BATCHES.items():
    b = ROOT / 'drive_batches' / name
    hashes = {str(p): hash_file(p) for p in b.rglob('*') if p.is_file()}
    sources.update(hashes)
    target = OUT / f'scenario{n}'
    target.mkdir(exist_ok=True)
    result_path = next((b/'results').glob('*/summary.csv'))
    rows = list(csv.DictReader(result_path.open()))
    settings = json.loads((result_path.parent/'settings.json').read_text())
    session = json.loads((b/'session_result.json').read_text())
    config = yaml.safe_load((b/'effective_config.yaml').read_text())
    meta = yaml.safe_load((b/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    assert len(rows) == 5 and session['status']['phase'] == 'done' and not session['failure']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(b/'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    selected = {'/knu_batch/status', '/motive/vehicle/odom_map', '/debug/lower_mpc_trace',
                '/debug/upper_branch_event', '/debug/upper_stage_timing', '/debug/upper_interval_centering_state'}
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in selected}
    counts = Counter()
    phases, odom, lower, events, cycles, bs = [], [], [], [], [], []
    previous = None
    while reader.has_next():
        topic, blob, stamp = reader.read_next()
        received = stamp/1e9
        counts[topic] += 1
        if topic not in selected:
            continue  # No grid decoding or retained grid payloads.
        msg = deserialize_message(blob, types[topic])
        if topic == '/knu_batch/status':
            value = json.loads(msg.data)
            key = (value['index'], value['phase'], value['sequence'])
            if key != previous:
                phases.append(dict(received_t=received, **value))
                previous = key
        elif topic == '/motive/vehicle/odom_map':
            odom.append([received, msg.pose.pose.position.x, msg.pose.pose.position.y,
                         msg.pose.pose.orientation.z,
                         math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)])
        elif topic == '/debug/lower_mpc_trace':
            lower.append([received, *msg.data])
        elif topic == '/debug/upper_branch_event':
            events.append(dict(received_t=received, **json.loads(msg.data)))
        elif topic == '/debug/upper_stage_timing':
            cycles.append(dict(received_t=received, **json.loads(msg.data)))
        else:
            bs.append(list(msg.data))
    expected = {t['topic_metadata']['name']: t['message_count'] for t in meta['topics_with_message_count'] if t['message_count']}
    assert dict(counts) == expected
    odom, lower, bs = np.asarray(odom), np.asarray(lower), np.asarray(bs)
    np.savez_compressed(target/'signals.npz', odom=odom, lower=lower, B=bs)
    with gzip.open(target/'events.json.gz', 'wt') as f:
        json.dump(events, f)
    with gzip.open(target/'timing.json.gz', 'wt') as f:
        json.dump(cycles, f)
    dump(target/'phases.json', phases)
    tracks, attempts = {}, []
    for i, row in enumerate(rows, 1):
        states = [p for p in phases if p['index'] == i]
        start = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'running')
        end = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'coasting')
        reference = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'reference')
        stop = next((p['phase_stamp_sec'] for p in phases if p['index'] == i+1), session['status']['phase_stamp_sec'])
        tr = odom[(odom[:, 0] >= start) & (odom[:, 0] < stop)]
        run = tr[:, 0] < end
        l = lower[(lower[:, 0] >= start) & (lower[:, 0] < end)]
        valid = ((l[:, 11] > 0) | (l[:, 28] > 0)) & np.isin(l[:, 13], [0, 1, 2, 5])
        ev = [e for e in events if start <= e['t_start'] < end]
        cy = [c for c in cycles if start <= c['t_start'] < end]
        bstates = bs[(bs[:, 0] >= start) & (bs[:, 0] < end)]
        first = next((e for e in events if reference <= e['t_start'] < start), None)
        initial_b = bs[np.argmin(abs(bs[:, 0]-first['t_emit']))] if first else None
        detail = json.loads(row['detail'])
        a = dict(scenario=n, attempt=i, outcome=row['outcome'], start_epoch_s=start,
                 outcome_epoch_s=end, stop_phase_epoch_s=stop, running_duration_s=end-start,
                 result_duration_s=float(row['duration_sec']), progress_pct=100*float(row['progress_ratio']),
                 cross_track_m=float(row['cross_track_m']), goal_distance_at_outcome_m=float(row['goal_distance_m']),
                 stop_status=row['stop_status'], stop_x=float(row['stop_x']), stop_y=float(row['stop_y']),
                 stop_goal_distance_m=float(row['stop_goal_distance_m']), running_samples=int(run.sum()),
                 running_distance_m=float(np.linalg.norm(np.diff(tr[run, 1:3], axis=0), axis=1).sum()),
                 upper_planning_cycles=len(ev), upper_unpublished=sum(not e['valid_plan'] for e in ev),
                 upper_failure_reasons=json.dumps(dict(Counter(e['reason'] for e in ev if not e['valid_plan']))),
                 upper_wall_max_ms=max(c['wall_ms'] for c in cy),
                 upper_receive_gap_max_s=max_gap([c['received_t'] for c in cy]),
                 lower_receive_gap_max_s=max_gap(l[:, 0]), lower_build_solver_return_max_ms=float(l[:, 12].max()),
                 lower_heading_rmse_deg=float(np.sqrt(np.mean(np.degrees(l[valid, 8])**2))),
                 lower_lateral_rmse_m=float(np.sqrt(np.mean(l[valid, 7]**2))),
                 B_min_m=float(bstates[:, 2].min()), B_max_m=float(bstates[:, 2].max()),
                 first_wp0=first['wp0'] if first else None, first_wp1=first['wp1'] if first else None,
                 first_B_before_m=float(initial_b[1]) if initial_b is not None else None,
                 road_evidence_confirmed=detail['road_evidence']['confirmed'],
                 road_evidence_reason=detail['road_evidence']['reason'])
        attempts.append(a)
        tracks[i] = tr
    all_attempts.extend(attempts)
    output_csv(target/'per_attempt.csv', attempts)
    route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader((b/f'routes/scenario{n}.csv').open())])
    points = np.vstack([route, *[tr[:, 1:3] for tr in tracks.values()]])
    bounds = points.min(0)-8, points.max(0)+8
    def draw(ax, indices):
        ax.plot(route[:, 0], route[:, 1], 'k--o', lw=1, ms=3)
        ax.plot(*route[0], 'k^', ms=7)
        ax.plot(*route[-1], 'k*', ms=9)
        for i in indices:
            a, tr, color = attempts[i-1], tracks[i], COLORS[i-1]
            run = tr[:, 0] < a['outcome_epoch_s']
            ax.plot(tr[run, 1], tr[run, 2], color=color, lw=1.5, label=f'{i}회 '+('목표 접근' if a['outcome']=='completed' else a['outcome']))
            if (~run).any():
                ax.plot(tr[~run, 1], tr[~run, 2], ':', color=color, lw=1.1)
            ax.plot(*tr[run][-1, 1:3], 'x', color=color, ms=7)
            ax.plot(a['stop_x'], a['stop_y'], 's', color=color, mfc='white', ms=5)
        ax.set_xlim(bounds[0][0], bounds[1][0])
        ax.set_ylim(bounds[0][1], bounds[1][1])
        ax.set_aspect('equal')
        ax.set_xlabel('map X [m]')
        ax.set_ylabel('map Y [m]')
        ax.grid(alpha=.2)
    fig, axes = plt.subplots(2, 3, figsize=(14, 10))
    draw(axes.flat[0], range(1, 6))
    axes.flat[0].set_title(f'시나리오{n} · 5회 전체')
    axes.flat[0].legend(fontsize=7, loc='best')
    for i, ax in enumerate(list(axes.flat)[1:], 1):
        draw(ax, [i])
        a = attempts[i-1]
        ax.set_title(f'{i}회 · '+('목표 접근' if a['outcome']=='completed' else a['outcome'])+
                     f' · 진행 {a["progress_pct"]:.1f}%\n목표 거리: 판정 {a["goal_distance_at_outcome_m"]:.1f}m → 정지 {a["stop_goal_distance_m"]:.1f}m')
    completed = sum(a['outcome']=='completed' for a in attempts)
    fig.suptitle(f'2026-10-13 시나리오{n} · 목표 접근 {completed}/5 · off_route {5-completed}/5\n실선: running · 점선: 관성 이동 · ×: 결과 판정 · □: 정지 확인 · ★: waypoint 목표', fontsize=13)
    fig.subplots_adjust(top=.86, bottom=.07, hspace=.35, wspace=.28)
    fig.savefig(OUT/f'scenario{n}_all5.png', dpi=160)
    fig.savefig(OUT/f'scenario{n}_all5.pdf')
    figures.append(fig)
    summary = dict(scenario=n, batch=name, attempts=5, completed=completed, off_route=5-completed,
                   stationary_confirmed=sum(a['stop_status']=='stationary' for a in attempts),
                   compute_watchdog_enabled=settings['require_compute_heartbeat'],
                   upper_wall_max_ms=max(a['upper_wall_max_ms'] for a in attempts),
                   upper_gap_max_s=max(a['upper_receive_gap_max_s'] for a in attempts),
                   lower_gap_max_s=max(a['lower_receive_gap_max_s'] for a in attempts),
                   bag_messages=sum(counts.values()), grid_decoded=0,
                   soft_ratio=config['upper_planner_node']['ros__parameters']['preview_interval_soft_ratio'],
                   waypoint_bias_enabled=config['upper_planner_node']['ros__parameters']['enable_waypoint_bias'])
    all_summaries.append(summary)
    dump(target/'summary.json', summary)
    batch_data[n] = dict(route=route, tracks=tracks, attempts=attempts)
    print(f'시나리오{n}: 목표 접근{completed}/5, 원본기록수 검증 완료', flush=True)

overview, axes = plt.subplots(1, 1, figsize=(6, 7))
for ax, n in zip(np.atleast_1d(axes), BATCHES):
    data = batch_data[n]
    route = data['route']
    ax.plot(route[:, 0], route[:, 1], 'k--o', ms=3, lw=1)
    ax.plot(*route[-1], 'k*', ms=9)
    points = [route]
    for a in data['attempts']:
        tr = data['tracks'][a['attempt']]
        running = tr[:, 0] < a['outcome_epoch_s']
        color = COLORS[a['attempt']-1]
        ax.plot(tr[running, 1], tr[running, 2], color=color, lw=1.5, label=f'{a["attempt"]}회')
        ax.plot(tr[~running, 1], tr[~running, 2], ':', color=color, lw=1)
        ax.plot(a['stop_x'], a['stop_y'], 's', color=color, mfc='white', ms=4)
        points.append(tr[:, 1:3])
    points = np.vstack(points)
    ax.set_xlim(points[:, 0].min()-8, points[:, 0].max()+8)
    ax.set_ylim(points[:, 1].min()-8, points[:, 1].max()+8)
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    s = next(s for s in all_summaries if s['scenario']==n)
    ax.set_title(f'시나리오{n} · 목표 접근 {s["completed"]}/5')
    ax.set_xlabel('map X [m]')
    ax.set_ylabel('map Y [m]')
    ax.legend(fontsize=7)
overview.suptitle('최신 로컬 기록: 시나리오3 · dt0.5·예측7단계 · 총5회\n검은 점선 waypoint · 색 실선 running · 점선 관성 이동 · □ 정지 · ★ 목표', fontsize=13)
overview.tight_layout(rect=(0, 0, 1, .90))
overview.savefig(OUT/'scenario3_overview.png', dpi=175)
with PdfPages(OUT/'SCENARIO3_DT05_H7_REVIEW.pdf') as pdf:
    pdf.savefig(overview)
    for f in figures:
        pdf.savefig(f)
plt.close('all')
output_csv(OUT/'per_attempt.csv', all_attempts)
dump(OUT/'summary.json', all_summaries)
assert all(hash_file(Path(p))==h for p,h in sources.items())
dump(OUT/'sources_sha256.json', sources)
dump(OUT/'validation.json', dict(original_inputs_preserved=True, attempts=5,
                                bag_record_counts_match=True, grid_decoded=0,
                                operating_nodes_started=False, code_or_settings_changed=False))
