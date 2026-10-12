"""Plot each latest five-run batch using saved odometry and phase records only."""
from pathlib import Path
from collections import Counter
import csv
import gzip
import hashlib
import json
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s12_latest_tracks_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s12_latest_tracks_mpl'
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.lines import Line2D

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCHES = {1: '20261012_212933_scenario1_c4b528', 2: '20261012_213850_scenario2_c672e5'}
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
LABELS = {'completed': '목표 접근', 'off_route': 'off_route', 'stopped_10s': 'stopped_10s'}
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 10})

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()

sources, validations = {}, []
for n, name in BATCHES.items():
    batch = ROOT / 'drive_batches' / name
    source_files = [p for p in batch.rglob('*') if p.is_file()]
    hashes = {str(p): sha(p) for p in source_files}
    summary_path = next((batch / 'results').glob('*/summary.csv'))
    rows = list(csv.DictReader(summary_path.open()))
    session = json.loads((batch / 'session_result.json').read_text())
    assert len(rows) == 5 and session['status']['phase'] == 'done' and not session['failure']
    assert all(r['stop_status'] == 'stationary' for r in rows)
    route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader((batch / f'routes/scenario{n}.csv').open())])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(batch / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/knu_batch/status', '/motive/vehicle/odom_map']))
    odom, phases = [], []
    previous = None
    counts = Counter()
    while reader.has_next():
        topic, blob, stamp = reader.read_next()
        counts[topic] += 1
        if topic == '/knu_batch/status':
            value = json.loads(deserialize_message(blob, String).data)
            key = (value['index'], value['phase'], value['sequence'])
            if key != previous:
                phases.append(value)
                previous = key
        else:
            msg = deserialize_message(blob, Odometry)
            odom.append([stamp / 1e9, msg.pose.pose.position.x, msg.pose.pose.position.y])
    metadata = yaml.safe_load((batch / 'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {t['topic_metadata']['name']: t['message_count'] for t in metadata['topics_with_message_count']}
    assert all(counts[t] == expected[t] for t in counts)
    odom = np.array(odom)
    tracks, details = [], []
    for i, row in enumerate(rows, 1):
        states = [p for p in phases if p['index'] == i]
        start = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'running')
        end = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'coasting')
        stop = next((p['phase_stamp_sec'] for p in phases if p['index'] == i + 1), session['status']['phase_stamp_sec'])
        track = odom[(odom[:, 0] >= start) & (odom[:, 0] < stop)]
        running = track[:, 0] < end
        assert running.any()
        tracks.append((track, running, row))
        details.append(dict(attempt=i, outcome=row['outcome'], start_epoch_s=start, outcome_epoch_s=end,
                            stationary_phase_epoch_s=stop, running_phase_duration_s=end-start,
                            stop_status=row['stop_status'], stop_goal_distance_m=float(row['stop_goal_distance_m'])))
    fig, ax = plt.subplots(figsize=(9, 8) if n == 1 else (13, 5.8))
    ax.plot(route[:, 0], route[:, 1], 'k--o', lw=1.2, ms=4, zorder=2)
    for i, (track, running, row) in enumerate(tracks, 1):
        color = COLORS[i-1]
        ax.plot(track[running, 1], track[running, 2], color=color, lw=1.6, alpha=.9)
        coast = track[~running]
        if len(coast):
            ax.plot(coast[:, 1], coast[:, 2], ':', color=color, lw=1.3)
        ax.plot(*track[running][-1, 1:3], marker='x', color=color, ms=8, mew=1.7)
        ax.plot(float(row['stop_x']), float(row['stop_y']), marker='s', color=color, mfc='white', ms=6, mew=1.4)
    ax.plot(*route[0], '^', color='black', ms=9)
    ax.plot(*route[-1], '*', color='black', ms=12)
    points = np.vstack([route, *[track[:, 1:3] for track, _, _ in tracks]])
    pad = np.maximum((points.max(0) - points.min(0)) * .06, 5)
    ax.set_xlim(points[:, 0].min() - pad[0], points[:, 0].max() + pad[0])
    ax.set_ylim(points[:, 1].min() - pad[1], points[:, 1].max() + pad[1])
    ax.set_aspect('equal')
    ax.set_xlabel('map X [m]')
    ax.set_ylabel('map Y [m]')
    ax.grid(alpha=.2)
    outcomes = Counter(r['outcome'] for r in rows)
    result = ' / '.join(f'{LABELS[k]} {v}회' for k, v in outcomes.items())
    fig.suptitle(f'시나리오 {n} · 최근 5회 궤적\n2026-10-12 {"21:29" if n == 1 else "21:38"} 배치 · {result}', fontsize=15, y=.98)
    handles = [Line2D([], [], color=c, lw=2, label=f'{i}회: {LABELS[r["outcome"]]}') for i, (c, r) in enumerate(zip(COLORS, rows), 1)]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5, .10), ncol=3, fontsize=9)
    fig.text(.04, .02,
             f'검은 점선: 원래 waypoint · △ 출발 · ★ 목표 · 색 실선: running · 점선: 관성 이동 · × 판정 · □ 정지 확인\n'
             f'completed는 목표 접근이며 목표 정차 성공이 아닙니다. 원본 배치: {name}', fontsize=8)
    fig.subplots_adjust(top=.86, bottom=.25, left=.10, right=.97)
    fig.savefig(OUT / f'scenario{n}_latest5.png', dpi=180)
    fig.savefig(OUT / f'scenario{n}_latest5.pdf')
    plt.close(fig)
    (OUT / f'scenario{n}_attempts.json').write_text(json.dumps(details, indent=2, ensure_ascii=False) + '\n')
    with gzip.open(OUT / f'scenario{n}_odom.csv.gz', 'wt', newline='') as f:
        w = csv.writer(f)
        w.writerow(['received_epoch_s', 'x_map_m', 'y_map_m'])
        w.writerows(odom)
    assert all(sha(Path(p)) == h for p, h in hashes.items())
    sources.update(hashes)
    validations.append(dict(scenario=n, batch=name, outcomes=dict(outcomes), departed=5,
                            stationary_confirmed=5, original_files_preserved=True,
                            selected_topic_record_counts_match_metadata=True, grids_decoded=0))
    print(f'시나리오{n} 저장 완료: {result}', flush=True)
(OUT / 'sources_sha256.json').write_text(json.dumps(sources, indent=2) + '\n')
(OUT / 'validation.json').write_text(json.dumps(validations, indent=2, ensure_ascii=False) + '\n')
