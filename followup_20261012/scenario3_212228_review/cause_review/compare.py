"""Saved-run branch comparison, retaining only two selected BEV messages."""
from pathlib import Path
import csv
import gzip
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s3_cause_review_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s3_cause_review_mpl'
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCH = ROOT / 'drive_batches/20261012_212228_scenario3_ebf13f'
events = json.load(gzip.open(OUT / 'events.json.gz', 'rt'))
attempts = list(csv.DictReader((OUT.parent / 'per_attempt.csv').open()))
telemetry = np.load(OUT / 'telemetry.npz')
bs = telemetry['B']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(BATCH / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
selected = []
fig, axes = plt.subplots(1, 2, figsize=(13, 7))
for ax, i in zip(axes, [2, 3]):
    a = attempts[i-1]
    es = [e for e in events if float(a['start_epoch_s']) <= e['t_start'] < float(a['outcome_epoch_s']) and 228 < e['x'] < 265 and e['y'] < 150]
    e = min(es, key=lambda e: abs(e['x'] - 239))
    reader.seek(int((e['t_start'] - 1) * 1e9))
    last = None
    while reader.has_next():
        _, blob, t = reader.read_next()
        if t > int(e['t_start'] * 1e9):
            break
        last = blob, t
    assert last is not None
    blob, t = last
    grid = deserialize_message(blob, OccupancyGrid)
    data = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    # Grid origin uses standard quaternion; planner pose yaw uses scalar odometry yaw.
    q = grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    gx, gy = np.meshgrid(np.arange(grid.info.width + 1) * grid.info.resolution, np.arange(grid.info.height + 1) * grid.info.resolution)
    bx = grid.info.origin.position.x + c*gx - s*gy
    by = grid.info.origin.position.y + s*gx + c*gy
    c, s = math.cos(e['yaw_rad']), math.sin(e['yaw_rad'])
    wx, wy = e['x'] + c*bx - s*by, e['y'] + s*bx + c*by
    categories = np.where(data < 0, 2, np.where(data <= 1, 0, 1))
    ax.pcolormesh(wx, wy, categories, cmap=ListedColormap(['white', '#575757', '#e8cf8b']), norm=BoundaryNorm([-.5, .5, 1.5, 2.5], 3), shading='flat', rasterized=True)
    for w in e['intervals_world']:
        k, candidate, x0, y0, x1, y1 = w[:6]
        color = '#ae2bca' if k == 6 else '#159b61'
        ax.plot([x0, x1], [y0, y1], color=color, lw=3 if k == 6 else 2)
        if k == 6:
            ax.text((x0+x1)/2 + .7, (y0+y1)/2 + .7, f'k6 후보 {int(candidate)}\n원래 폭 {w[6]:.1f}m', color=color, fontsize=9,
                    bbox=dict(facecolor='white', alpha=.9, edgecolor='none'))
    preview = np.array(e['preview_world'])
    plan = np.array(e['published_world'])
    ax.plot(preview[:, 0], preview[:, 1], '--o', color='#ff9a24', lw=1.4, ms=3)
    ax.plot(plan[:, 0], plan[:, 1], '-o', color='#d51b42', lw=2, ms=3)
    ax.plot(e['x'], e['y'], 'k^', ms=8)
    j = np.argmin(abs(bs[:, 0] - e['t_start']))
    ax.set_title(f'{i}회 · {"목표 접근" if i == 2 else "off_route"}\n차량 ({e["x"]:.2f}, {e["y"]:.2f}) · yaw {math.degrees(e["yaw_rad"]):+.1f}°\nB {bs[j,1]:.1f} → {bs[j,2]:.1f}m · k6 후보 {e["candidate_counts"][6]}개', fontsize=11)
    ax.set_xlim(229, 276)
    ax.set_ylim(124, 158)
    ax.set_aspect('equal')
    ax.set_facecolor('#e6eaf0')
    ax.grid(alpha=.12)
    ax.set_xlabel('map X [m]')
    ax.set_ylabel('map Y [m]')
    np.savez_compressed(OUT / f'selected_grid_a{i}.npz', data=data)
    selected.append(dict(attempt=i, event=e, grid_receive_epoch_s=t/1e9,
                         grid_before_plan_ms=1000*(e['t_start']-t/1e9),
                         grid_header_stamp_sec=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9))
fig.suptitle('같은 배치의 분기점 비교 — 북동쪽 후보 유무와 발행 계획', fontsize=15, y=.98)
fig.legend(handles=[Line2D([], [], color='#159b61', lw=2, label='처리 후 인터벌'),
                    Line2D([], [], color='#ae2bca', lw=3, label='k6 인터벌 후보'),
                    Line2D([], [], color='#ff9a24', ls='--', label='최적화 전 참조'),
                    Line2D([], [], color='#d51b42', lw=2, label='발행 계획')],
           loc='lower center', bbox_to_anchor=(.5, .10), ncol=4, fontsize=9)
fig.text(.04, .025,
         '각 배경은 해당 계획 시작 직전의 BEV 한 장입니다. 계획 주기의 차량 위치·yaw로 표시했으며 누적하지 않았습니다.\n'
         'BEV 헤더 시각=0, 계획 로그의 BEV 식별자 없음: 최신 선행 수신 기록과 대응했으며 실제 영상 획득 지연은 이 그림으로 측정할 수 없습니다.', fontsize=8)
fig.subplots_adjust(top=.78, bottom=.22, wspace=.15)
fig.savefig(OUT / 'branch_candidate_comparison.png', dpi=180)
fig.savefig(OUT / 'branch_candidate_comparison.pdf')
plt.close(fig)
(OUT / 'selected_cycles.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2) + '\n')

rows = []
for i in range(1, 6):
    a = attempts[i-1]
    for e in events:
        if not(float(a['start_epoch_s']) <= e['t_start'] < float(a['outcome_epoch_s']) and 228 < e['x'] < 268 and e['y'] < 150):
            continue
        j = np.argmin(abs(bs[:, 0] - e['t_start']))
        p = np.asarray(e['published_world'])
        rows.append(dict(attempt=i, event_epoch_s=e['t_start'], running_elapsed_s=e['t_start']-float(a['start_epoch_s']),
                         x=e['x'], y=e['y'], yaw_deg=math.degrees(e['yaw_rad']),
                         B_before_m=float(bs[j,1]), B_after_m=float(bs[j,2]), reason=e['reason'], status=e['status'],
                         retained_previous_path=e['retained_previous_path'], candidate_counts=json.dumps(e['candidate_counts']),
                         plan_tail_heading_deg=float(np.degrees(np.arctan2(*(p[-1]-p[-2])[::-1]))) if len(p) > 1 else None))
with (OUT / 'branch_timeline.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
print('후보 비교 그림과 분기 시계열 저장 완료')
