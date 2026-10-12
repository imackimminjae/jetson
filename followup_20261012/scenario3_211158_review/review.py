"""Read saved telemetry only; never decode or retain occupancy-grid payloads."""
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
os.environ['ROS_LOG_DIR'] = '/tmp/scenario3_211158_review_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/scenario3_211158_review_mpl'
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
BATCH = ROOT / 'drive_batches/20261012_211158_scenario3_611774'
OUT = Path(__file__).resolve().parent

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()

sources = {str(p): sha(p) for p in BATCH.rglob('*') if p.is_file()}
summary_path = next((BATCH / 'results').glob('*/summary.csv'))
rows = list(csv.DictReader(summary_path.open()))
settings = json.loads((summary_path.parent / 'settings.json').read_text())
session = json.loads((BATCH / 'session_result.json').read_text())
config = yaml.safe_load((BATCH / 'effective_config.yaml').read_text())
meta = yaml.safe_load((BATCH / 'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(BATCH / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
selected = {'/knu_batch/status', '/motive/vehicle/odom_map', '/debug/lower_mpc_trace',
            '/debug/upper_branch_event', '/debug/upper_stage_timing', '/debug/upper_interval_centering_state'}
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in selected}
counts = Counter()
phases, odom, lower, events, timing, bstates = [], [], [], [], [], []
last_phase = None
while reader.has_next():
    topic, blob, stamp = reader.read_next()
    t = stamp / 1e9
    counts[topic] += 1
    if topic not in selected:
        continue  # Includes every grid message: count serialized records only.
    msg = deserialize_message(blob, types[topic])
    if topic == '/knu_batch/status':
        value = json.loads(msg.data)
        key = (value['index'], value['phase'], value['sequence'])
        if key != last_phase:
            phases.append(dict(received_t=t, **value))
            last_phase = key
    elif topic == '/motive/vehicle/odom_map':
        odom.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.orientation.z,
                     math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)])
    elif topic == '/debug/lower_mpc_trace':
        lower.append([t, *msg.data])
    elif topic == '/debug/upper_branch_event':
        value = json.loads(msg.data)
        events.append({k: value.get(k) for k in ('t_start', 't_emit', 'wp0', 'wp1', 'valid_plan', 'reason', 'status')})
    elif topic == '/debug/upper_stage_timing':
        timing.append(dict(received_t=t, **json.loads(msg.data)))
    else:
        bstates.append(list(msg.data))

expected_counts = {t['topic_metadata']['name']: t['message_count'] for t in meta['topics_with_message_count']}
assert dict(counts) == {k: v for k, v in expected_counts.items() if v > 0}, (counts, expected_counts)
assert sum(counts.values()) == meta['message_count']
assert len(rows) == 5 and session['status']['phase'] == 'done' and not session['failure']
odom = np.asarray(odom)
lower = np.asarray(lower)
bstates = np.asarray(bstates)

def write_csv(path, values):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'wt', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(values[0]))
        writer.writeheader()
        writer.writerows(values)

def gap(values):
    values = np.asarray(values)
    return float(np.max(np.diff(values))) if len(values) > 1 else None

attempts, tracks, active_traces = [], {}, {}
for index, row in enumerate(rows, 1):
    states = [p for p in phases if p['index'] == index]
    start = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'running')
    end = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'coasting')
    reference = next(p['phase_stamp_sec'] for p in states if p['phase'] == 'reference')
    stop = next((p['phase_stamp_sec'] for p in phases if p['index'] == index + 1),
                session['status']['phase_stamp_sec'])
    running = odom[(odom[:, 0] >= start) & (odom[:, 0] < end)]
    tr = odom[(odom[:, 0] >= start) & (odom[:, 0] < stop)]
    traces = lower[(lower[:, 0] >= start) & (lower[:, 0] < end)]
    # Columns after removing bag receive time retain the documented trace indices.
    valid = ((traces[:, 11] > 0) | (traces[:, 28] > 0)) & np.isin(traces[:, 13], [0, 1, 2, 5])
    ev = [v for v in events if start <= v['t_start'] < end]
    cycles = [v for v in timing if start <= v['t_start'] < end]
    first = next((v for v in events if reference <= v['t_start'] < start), None)
    initial_b = bstates[np.argmin(abs(bstates[:, 0] - first['t_emit']))] if first else None
    detail = json.loads(row['detail'])
    r = dict(attempt=index, outcome=row['outcome'], duration_sec=float(row['duration_sec']),
             progress_pct=100 * float(row['progress_ratio']),
             waypoint_cross_track_m=float(row['cross_track_m']),
             goal_distance_at_outcome_m=float(row['goal_distance_m']),
             stop_status=row['stop_status'], stop_x=float(row['stop_x']), stop_y=float(row['stop_y']),
             goal_distance_at_stop_m=float(row['stop_goal_distance_m']),
             start_kst=datetime.fromtimestamp(start, ZoneInfo('Asia/Seoul')).isoformat(timespec='milliseconds'),
             start_epoch_s=start, outcome_epoch_s=end, stop_phase_epoch_s=stop,
             running_phase_duration_sec=end-start,
             running_samples=len(running), max_running_speed_mps=float(running[:, 4].max()),
             running_distance_m=float(np.linalg.norm(np.diff(running[:, 1:3], axis=0), axis=1).sum()),
             upper_plan_attempts=len(ev), upper_unpublished_count=sum(not v['valid_plan'] for v in ev),
             upper_unpublished_reasons=json.dumps(dict(Counter(v['reason'] for v in ev if not v['valid_plan'])), ensure_ascii=False),
             upper_wall_max_ms=max(v['wall_ms'] for v in cycles),
             upper_received_max_gap_s=gap([v['received_t'] for v in cycles]),
             lower_received_max_gap_s=gap(traces[:, 0]),
             lower_build_solver_return_max_ms=float(np.max(traces[:, 12])),
             heading_valid_samples=int(valid.sum()),
             body_heading_rmse_deg=float(np.sqrt(np.mean(np.degrees(traces[valid, 8]) ** 2))),
             first_reference_wp0=first['wp0'] if first else None,
             first_reference_wp1=first['wp1'] if first else None,
             first_B_before_m=float(initial_b[1]) if initial_b is not None else None,
             road_evidence_confirmed=detail['road_evidence']['confirmed'],
             road_evidence_reason=detail['road_evidence']['reason'])
    attempts.append(r)
    tracks[index] = tr
    active_traces[index] = (traces, valid)
write_csv(OUT / 'per_attempt.csv', attempts)
write_csv(OUT / 'odom.csv.gz', [dict(zip(['received_epoch_s', 'x_map_m', 'y_map_m', 'scalar_yaw_rad', 'speed_mps'], v)) for v in odom])
(OUT / 'phases.json').write_text(json.dumps(phases, ensure_ascii=False, indent=2) + '\n')

route = np.array([[float(v['x']), float(v['y'])] for v in csv.DictReader((BATCH / 'routes/scenario3.csv').open())])
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 9})
colors = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
labels = {'completed': '목표 접근', 'off_route': 'off_route'}
points = np.vstack([route, *[v[:, 1:3] for v in tracks.values()]])
minimum, maximum = points.min(0) - 8, points.max(0) + 8

def draw(ax, indices):
    ax.plot(route[:, 0], route[:, 1], 'k--o', lw=1, ms=3, label='원래 waypoint')
    ax.plot(*route[0], '^', color='black', ms=8)
    ax.plot(*route[-1], '*', color='black', ms=10, label='목표')
    for i in indices:
        a, tr, color = attempts[i-1], tracks[i], colors[i-1]
        run = tr[:, 0] < a['outcome_epoch_s']
        ax.plot(tr[run, 1], tr[run, 2], color=color, lw=1.6, label=f'{i}회 {labels[a["outcome"]]}')
        coast = tr[~run]
        if len(coast):
            ax.plot(coast[:, 1], coast[:, 2], ':', color=color, lw=1.3)
        ax.plot(tr[run][-1, 1], tr[run][-1, 2], 'x', color=color, ms=7)
        ax.plot(a['stop_x'], a['stop_y'], 's', color=color, markerfacecolor='white', ms=5)
    ax.set_xlim(minimum[0], maximum[0])
    ax.set_ylim(minimum[1], maximum[1])
    ax.set_aspect('equal')
    ax.grid(alpha=.2)
    ax.set_xlabel('map X [m]')
    ax.set_ylabel('map Y [m]')

fig, axes = plt.subplots(2, 3, figsize=(13, 9))
draw(axes.flat[0], range(1, 6))
axes.flat[0].set_title('전체 5회 궤적')
axes.flat[0].legend(fontsize=7, loc='upper right')
for i, ax in enumerate(list(axes.flat)[1:], 1):
    draw(ax, [i])
    a = attempts[i-1]
    ax.set_title(f'{i}회 · {labels[a["outcome"]]} · running {a["running_phase_duration_sec"]:.1f}초\n목표 거리: 판정 {a["goal_distance_at_outcome_m"]:.1f}m → 정지 {a["goal_distance_at_stop_m"]:.1f}m')
fig.suptitle('2026-10-12 시나리오3 · 5회 결과: 목표 접근 2 / off_route 3\n실선: running · 점선: 관성 이동 · ×: 결과 판정 · □: 정지 확인 위치', fontsize=13)
fig.tight_layout(rect=(0, .02, 1, .92))
fig.subplots_adjust(hspace=.42, wspace=.28)
fig.savefig(OUT / 'scenario3_all5.png', dpi=170)
fig.savefig(OUT / 'scenario3_all5.pdf')

heading, axes_h = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
for i in range(1, 6):
    a, tr = attempts[i-1], tracks[i]
    run = tr[:, 0] < a['outcome_epoch_s']
    axes_h[0].plot(tr[run, 0] - a['start_epoch_s'], np.degrees(np.unwrap(tr[run, 3])), color=colors[i-1], label=f'{i}회 {labels[a["outcome"]]}')
    traces, valid = active_traces[i]
    values = np.full(len(traces), np.nan)
    values[valid] = np.degrees(traces[valid, 8])
    axes_h[1].plot(traces[:, 0] - a['start_epoch_s'], values, color=colors[i-1])
axes_h[0].set_ylabel('map 차량 yaw [°]')
axes_h[1].set_ylabel('하위 참조 기준\nbody heading error [°]')
axes_h[1].set_xlabel('running 시작 후 [초]')
for ax in axes_h:
    ax.grid(alpha=.2)
axes_h[0].legend(ncol=5, fontsize=8)
heading.suptitle('시나리오3 5회 헤딩 기록 — yaw는 회차별 unwrap\n오차는 유효 QP 또는 피드백이 있는 mode 0/1/2/5 표본만 표시', fontsize=12)
heading.tight_layout(rect=(0, 0, 1, .92))
heading.savefig(OUT / 'scenario3_heading.png', dpi=160)
with PdfPages(OUT / 'SCENARIO3_5_RUNS_REVIEW.pdf') as pdf:
    pdf.savefig(fig)
    pdf.savefig(heading)
plt.close('all')

validation = dict(original_files_preserved=all(sha(Path(p)) == h for p, h in sources.items()),
                  bag_record_count_matches_metadata=True, message_counts=dict(counts),
                  scheduled=5, running_confirmed=sum(a['running_samples'] > 0 for a in attempts),
                  stationary_confirmed=sum(a['stop_status'] == 'stationary' for a in attempts),
                  outcomes=dict(Counter(a['outcome'] for a in attempts)),
                  session_failure=session['failure'], grid_messages_decoded=0,
                  compute_watchdog_enabled=settings['require_compute_heartbeat'],
                  monitoring_scope='descriptive saved-log review; no cause analysis or tuning')
assert validation['original_files_preserved']
(OUT / 'validation.json').write_text(json.dumps(validation, ensure_ascii=False, indent=2) + '\n')
(OUT / 'sources_sha256.json').write_text(json.dumps(sources, indent=2) + '\n')
upper_config = config['upper_planner_node']['ros__parameters']
lines = [
    '2026-10-12 시나리오3 5회 저장 로그 확인',
    f'배치: {BATCH}',
    f'원본 결과: {summary_path}',
    '목표 접근 2회(2·4), off_route 3회(1·3·5), 시스템 스킵 0.',
    '5회 모두 running 기록 및 stationary 결과 확인. session done, failure 빈 문자열.',
    f'기록기 정상 종료 및 MCAP 전체 {meta["message_count"]}개 기록 수 메타데이터와 일치.',
    'completed는 목표 접근 판정이며 목표 정차 성공이 아니다.',
    '회차 | 결과 | 결과기록 duration_sec | waypoint 횡거리(m) | 목표거리 판정→정지(m)',
]
for a in attempts:
    lines.append(f'{a["attempt"]} | {a["outcome"]} | {a["duration_sec"]:.3f} | {a["waypoint_cross_track_m"]:.4f} | {a["goal_distance_at_outcome_m"]:.4f} → {a["goal_distance_at_stop_m"]:.4f}')
lines += [
    f'실행 설정: soft_ratio={upper_config["preview_interval_soft_ratio"]}, enable_waypoint_bias={upper_config["enable_waypoint_bias"]}.',
    f'waypoint_yaw_bias_rad={upper_config["waypoint_yaw_bias_rad"]}은 저장 값일 뿐 enable=false로 적용 안 됨.',
    f'road_route_v1 활성; route_unavailable 확인 {sum(a["road_evidence_confirmed"] for a in attempts)}회.',
    '단독 실행 설정 require_compute_heartbeat=false. 상위5초/하위2초 자동 watchdog은 활성화되지 않았음.',
    '그림 시간은 running 상태 시작→coasting 전환이며 결과기록 duration_sec와 별도로 계산.',
    '수신 공백/처리 시간 통계는 per_attempt.csv. 수신 공백만으로 계산 중단 원인을 확정하지 않음.',
    '그림 배경 누적 지도 없음. 원래 waypoint, running, 관성 이동, 정지 위치만 표시.',
    'odom orientation.z scalar yaw. 헤딩 오차는 하위 참조 기준이며 EKF–SIH 차이와 별개.',
    'occupancy grid는 직렬화된 기록 개수만 집계; 디코딩/캐시 없음.',
    '원본 로그/기존 미커밋 변경/제어·PX4 설정 변경 없음. 재주행/ARM/ROS publish 없음.',
    '이 확인은 결과와 기록 상태 정리이며 원인 분석·튜닝은 수행하지 않음.',
]
(OUT / 'RESULT_KO.txt').write_text('\n'.join(lines) + '\n')
print(json.dumps(dict(validation=validation, attempts=attempts), ensure_ascii=False, indent=2))
