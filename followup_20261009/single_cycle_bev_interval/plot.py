"""Render exactly one saved BEV frame with recorded-cycle intervals."""
from pathlib import Path
import csv
import json
import os
from datetime import datetime
from zoneinfo import ZoneInfo

os.environ['MPLCONFIGDIR'] = '/tmp/single_cycle_bev_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

OUT = Path(__file__).resolve().parent
e = json.loads((OUT / 'event.json').read_text())
m = json.loads((OUT / 'grid_metadata.json').read_text())
data = np.load(OUT / 'single_grid.npz')['data']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})

def rotation(yaw):
    return np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

pose = np.array([e['x'], e['y']])
pose_rotation = rotation(e['yaw_rad'])
grid_rotation = rotation(m['origin_yaw_rad'])
origin = np.array([m['origin_x'], m['origin_y']])

def world_to_grid(points):
    body = (np.asarray(points) - pose) @ pose_rotation
    return (body - origin) @ grid_rotation

def sample_values(points):
    indices = np.floor(world_to_grid(points) / m['resolution']).astype(int)
    valid = (indices[:, 0] >= 0) & (indices[:, 0] < m['width']) & (indices[:, 1] >= 0) & (indices[:, 1] < m['height'])
    values = np.full(len(indices), -999, dtype=int)
    values[valid] = data[indices[valid, 1], indices[valid, 0]]
    return values

local_x, local_y = np.meshgrid(np.arange(m['width'] + 1) * m['resolution'], np.arange(m['height'] + 1) * m['resolution'])
local = np.stack([local_x, local_y], axis=-1)
body = local @ grid_rotation.T + origin
world = body @ pose_rotation.T + pose
categories = np.where(data < 0, 2, np.where(data <= 1, 0, 1))
cmap = ListedColormap(['#ffffff', '#575757', '#e8cf8b'])
norm = BoundaryNorm([-.5, .5, 1.5, 2.5], cmap.N)
sections, rows = [], []
for w in e['intervals_world']:
    k, candidate, x0, y0, x1, y1, raw_width, width = w[:8]
    start, end = np.array([x0, y0]), np.array([x1, y1])
    assert np.isclose(np.linalg.norm(end - start), width, atol=1e-6)
    direction = (end - start) / width
    inset = (raw_width - width) / 2
    raw_start, raw_end = start - inset * direction, end + inset * direction
    sections.append((int(k), raw_start, raw_end, start, end, (start + end)/2, raw_width, width))
    for phase, p, q in [('raw_reconstructed', raw_start, raw_end), ('processed_recorded', start, end)]:
        # Dense display checks at 1 cm spacing; not an independent rerun of planner extraction.
        points = p + np.linspace(0, 1, int(np.ceil(np.linalg.norm(q - p)/.01)) + 1)[:, None] * (q - p)
        values = sample_values(points)
        endpoints = sample_values(np.array([p, q]))
        rows.append(dict(k=int(k), phase=phase, sample_spacing_max_m=.01,
                         sample_count=len(values), free_count=int(np.sum((values >= 0) & (values <= 1))),
                         blocked_count=int(np.sum(values > 1)), unknown_count=int(np.sum(values == -1)),
                         outside_count=int(np.sum(values == -999)), start_cell_value=int(endpoints[0]),
                         end_cell_value=int(endpoints[1])))

fig, axes = plt.subplots(1, 2, figsize=(13, 7.5), sharex=True, sharey=True)
for phase, ax in enumerate(axes):
    ax.pcolormesh(world[:, :, 0], world[:, :, 1], categories, cmap=cmap, norm=norm, shading='flat', rasterized=True)
    for k, raw_start, raw_end, start, end, midpoint, raw_width, width in sections:
        if phase == 0:
            p, q, color, display_width = raw_start, raw_end, '#2183d4', raw_width
        else:
            p, q, color, display_width = start, end, '#169d62', width
            ax.plot([raw_start[0], start[0]], [raw_start[1], start[1]], '--', color='#a5a5a5', lw=1.8)
            ax.plot([end[0], raw_end[0]], [end[1], raw_end[1]], '--', color='#a5a5a5', lw=1.8)
        ax.plot([p[0], q[0]], [p[1], q[1]], color=color, lw=2.8)
        ax.plot([p[0], q[0]], [p[1], q[1]], 'o', color=color, ms=3)
        ax.plot(*midpoint, 'o', c='#161616', ms=3)
        label_position = midpoint + np.array([.8, -.8])
        if k == 6:
            label_position = midpoint + np.array([-5.4, 1.6])
        ax.text(*label_position, f'k{k}: {display_width:.2f}m', color=color, fontsize=9,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.85, pad=1.2))
    plan = np.array(e['published_world'])
    ax.plot(plan[:, 0], plan[:, 1], color='#d51945', lw=2, zorder=5)
    ax.plot(*pose, 'k^', ms=8)
    ax.set_xlim(247, 279)
    ax.set_ylim(124, 166)
    ax.set_aspect('equal')
    ax.set_facecolor('#e6eaf0')
    ax.grid(alpha=.14)
    ax.set_xlabel('map X [m]')
    ax.set_title('단일 BEV + 축소 전 구간' if phase == 0 else '같은 BEV + 축소 후 구간', fontsize=13)
axes[0].set_ylabel('map Y [m]')
time = datetime.fromtimestamp(e['t_start'], ZoneInfo('Asia/Seoul')).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
fig.suptitle(f'기존 시나리오3 · 2회차 · {time} KST', fontsize=15, y=.965)
fig.legend(handles=[Patch(facecolor='#ffffff', edgecolor='#aaa', label='BEV 주행 가능'),
                    Patch(facecolor='#575757', label='BEV 비주행 영역'),
                    Patch(facecolor='#e6eaf0', label='BEV 범위 밖'),
                    Line2D([], [], color='#2183d4', lw=3, label='축소 전'),
                    Line2D([], [], color='#169d62', lw=3, label='축소 후'),
                    Line2D([], [], color='#d51945', lw=2, label='발행 계획')],
           loc='lower center', bbox_to_anchor=(.5, .105), ncol=6, fontsize=9)
fig.text(.04, .024,
         f'배경: 계획 시작 {m["age_before_plan_sec"] * 1000:.1f}ms 전에 기록된 BEV 한 장(128×128, 0.3125m 셀). 누적 지도 미사용.\n'
         'BEV는 해당 계획 주기의 차량 위치·yaw로 map 좌표에 표시했습니다. 축소 전 끝점은 기록된 원래 폭과 대칭 축소 규칙으로 복원했습니다.\n'
         '계획 로그에 사용한 BEV 메시지 ID가 없어 최신 선행 기록을 사용했습니다. 센서 시각은 0으로 저장되어 수신 기록 시각을 기준으로 선택했습니다.', fontsize=8)
fig.subplots_adjust(bottom=.24, top=.87, wspace=.08)
fig.savefig(OUT / 'single_cycle_bev_before_after.png', dpi=180)
fig.savefig(OUT / 'single_cycle_bev_before_after.pdf')
plt.close(fig)
with (OUT / 'interval_grid_checks.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
(OUT / 'validation.json').write_text(json.dumps(dict(
    grid_frames_retained=1, grid_bytes=int(data.nbytes), accumulation=False,
    code_or_operational_config_modified=False, checks=rows,
    interpretation='dense checks against selected saved grid only; not proof of planner callback message identity'), indent=2) + '\n')
print(json.dumps(rows, indent=2))
