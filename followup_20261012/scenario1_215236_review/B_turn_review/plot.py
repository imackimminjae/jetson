"""Observed B transitions and three single-frame turn snapshots; no tuning."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import csv
import gzip
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s1_B_turn_plot_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s1_B_turn_plot_mpl'
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCH = ROOT / 'drive_batches/20261012_215236_scenario1_9e41f2'
ats = json.loads((OUT.parent / 'scenario1_attempts.json').read_text())
es = json.load(gzip.open(OUT / 'events.json.gz', 'rt'))
telemetry = np.load(OUT / 'telemetry.npz')
bs, lower = telemetry['B'], telemetry['lower']
info = json.load(gzip.open(OUT / 'interval_info.json.gz', 'rt'))
it = np.array([t for t, _ in info])
odom = np.array([[float(r[k]) for k in ['received_epoch_s', 'x_map_m', 'y_map_m']]
                 for r in csv.DictReader(gzip.open(OUT.parent / 'scenario1_odom.csv.gz', 'rt'))])
route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader((BATCH / 'routes/scenario1.csv').open())])
colors = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})

def running_events(i):
    a = ats[i-1]
    return [e for e in es if a['start_epoch_s'] <= e['t_start'] < a['outcome_epoch_s']
            and -38 < e['y'] < -1 and e['x'] < 150]

timeline, ax = plt.subplots(figsize=(11, 4.8))
summaries = []
for i in range(1, 6):
    ev = running_events(i)
    b = np.array([bs[np.argmin(abs(bs[:, 0] - e['t_emit']))] for e in ev])
    x = np.array([e['x'] for e in ev])
    valid = np.array([e['valid_plan'] for e in ev])
    ax.step(x, b[:, 2], where='post', color=colors[i-1], lw=1.6, label=f'{i}회')
    ax.plot(x[~valid], b[~valid, 2], 'x', color=colors[i-1], ms=8, mew=2)
    summaries.append(dict(attempt=i, B_min=float(b[:, 2].min()), B_max=float(b[:, 2].max()),
                          upper_failures=int((~valid).sum())))
ax.set_xlabel('계획 주기 차량 map X [m]')
ax.set_ylabel('B [m]')
ax.grid(alpha=.2)
ax.legend(ncol=5)
ax.set_title('두 번째 회전 구간의 B 변화 · ×는 상위 계획 실패 주기')
ax.annotate('5회: B=1.7m 유지\n새 후보 5m대도 갱신 거부', xy=(134, 1.7), xytext=(129, 8),
            arrowprops=dict(arrowstyle='->', color='#976300'), color='#976300', fontsize=10)
timeline.tight_layout()
timeline.savefig(OUT / 'B_turn_all5.png', dpi=170)

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(BATCH / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
fig, axes = plt.subplots(1, 3, figsize=(15, 6.3), sharex=True, sharey=True)
selected = []
for panel, (ax, i, target) in enumerate(zip(axes, [1, 5, 5], [(113, -20), (114, -20), (122, -7)])):
    ev = running_events(i)
    e = min(ev, key=lambda e: (e['x']-target[0])**2 + (e['y']-target[1])**2)
    b = bs[np.argmin(abs(bs[:, 0] - e['t_emit']))]
    raw_info = np.array(info[np.argmin(abs(it - e['received_t']))][1]).reshape(-1, 12)
    reader.seek(int((e['t_start'] - 1) * 1e9))
    last = None
    while reader.has_next():
        _, blob, t = reader.read_next()
        if t > int(e['t_start'] * 1e9):
            break
        last = blob, t
    assert last is not None
    blob, stamp = last
    grid = deserialize_message(blob, OccupancyGrid)
    data = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    q = grid.info.origin.orientation
    gyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    gx, gy = np.meshgrid(np.arange(grid.info.width+1)*grid.info.resolution,
                         np.arange(grid.info.height+1)*grid.info.resolution)
    c, s = math.cos(gyaw), math.sin(gyaw)
    bx = grid.info.origin.position.x+c*gx-s*gy
    by = grid.info.origin.position.y+s*gx+c*gy
    c, s = math.cos(e['yaw_rad']), math.sin(e['yaw_rad'])
    wx, wy = e['x']+c*bx-s*by, e['y']+s*bx+c*by
    categories = np.where(data < 0, 2, np.where(data <= 1, 0, 1))
    ax.pcolormesh(wx, wy, categories, shading='flat', rasterized=True,
                  cmap=ListedColormap(['white', '#575757', '#e8cf8b']),
                  norm=BoundaryNorm([-.5, .5, 1.5, 2.5], 3))
    for w in e['intervals_world']:
        k, candidate, x0, y0, x1, y1 = w[:6]
        ax.plot([x0, x1], [y0, y1], color='#27935f', lw=2)
        if k in [1, 2] and candidate == 0:
            match = raw_info[(raw_info[:, 0] == k) & (raw_info[:, 1] == candidate)]
            mode = 'ON' if len(match) and match[0, 4] else 'OFF'
            ax.text((x0+x1)/2+.4, (y0+y1)/2+1.3, f'k{int(k)} {w[6]:.1f}→{w[7]:.1f}m {mode}',
                    fontsize=8, color='#116c39', bbox=dict(fc='white', alpha=.85, ec='none'))
    a = ats[i-1]
    tr = odom[(odom[:, 0] >= a['start_epoch_s']) & (odom[:, 0] < a['outcome_epoch_s'])]
    ax.plot(tr[:, 1], tr[:, 2], color=colors[i-1], lw=2)
    plan = np.asarray(e['published_world'] if e['published_world'] else e.get('retained_world', []))
    if len(plan):
        ax.plot(plan[:, 0], plan[:, 1], '--' if not e['valid_plan'] else '-', color='#cc1746', lw=1.6)
    ax.plot(route[:, 0], route[:, 1], 'k:', lw=1)
    ax.plot(e['x'], e['y'], 'k^', ms=7)
    ax.set_xlim(108, 151)
    ax.set_ylim(-25, 3)
    ax.set_aspect('equal')
    ax.set_facecolor('#e6eaf0')
    ax.grid(alpha=.15)
    ax.set_xlabel('map X [m]')
    ax.set_title(f'{i}회 · ({e["x"]:.1f}, {e["y"]:.1f})\nB {b[1]:.1f}→{b[2]:.1f}m · {"계획 발행" if e["valid_plan"] else "QP 실패·이전 계획 유지"}', fontsize=10)
    np.savez_compressed(OUT / f'turn_snapshot_{panel+1}.npz', data=data)
    selected.append(dict(attempt=i, event=e, B_state=b.tolist(), interval_info=raw_info.tolist(),
                         grid_receive_epoch_s=stamp/1e9, grid_before_plan_ms=1000*(e['t_start']-stamp/1e9)))
axes[0].set_ylabel('map Y [m]')
fig.suptitle('두 번째 회전: 계획에 이미 있는 굴곡과 B 감소 후 제약 변화', fontsize=14, y=.97)
fig.legend(handles=[Line2D([], [], color='#d39917', lw=2, label='실제 궤적(회차 색)'),
                    Line2D([], [], color='#cc1746', lw=2, label='상위 발행 계획'),
                    Line2D([], [], color='#cc1746', ls='--', label='유지한 이전 계획'),
                    Line2D([], [], color='#27935f', lw=2, label='처리 후 인터벌')],
           ncol=4, loc='lower center', bbox_to_anchor=(.5, .13), fontsize=9)
fig.text(.04, .035,
         '각 배경은 해당 계획 시작 직전 BEV 한 장을 계획 pose로 표시. 옅은 청회색은 BEV 밖이며 누적 지도 미사용.\n'
         '궤적은 구간 전체 기록이므로 배경과 모두 같은 시각이 아님. BEV 헤더 시각0·사용 메시지 ID 미기록. 차체의 도로 여유 검증이 아님.', fontsize=8)
fig.subplots_adjust(top=.82, bottom=.26, wspace=.12)
fig.savefig(OUT / 'turn_plan_B_snapshots.png', dpi=175)
with PdfPages(OUT / 'S1_TURN_B_REVIEW.pdf') as pdf:
    pdf.savefig(timeline)
    pdf.savefig(fig)
plt.close('all')
(OUT / 'selected_snapshots.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2) + '\n')
(OUT / 'B_summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(summaries, indent=2))
