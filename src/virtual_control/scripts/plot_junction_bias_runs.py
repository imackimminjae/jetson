#!/usr/bin/env python3
"""Plot actual driving traces and outcomes from audited short bias runs."""
import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/junction_driving_matplotlib')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D

WORKSPACE = Path(__file__).resolve().parents[3]
COLORS = {-10: '#c74747', -5: '#df9336', 0: '#252525', 5: '#2487b8', 10: '#8b60ad'}
LABELS = {26: '원형 교차로 · 서 → 동', 27: '원형 교차로 · 남 → 동',
          28: '남동 분기 · 서 → 동', 29: '남동 분기 · 남 → 북동',
          30: '북동 교차로 · 남서 → 동', 31: '북동 교차로 · 남서 → 북'}


def save(fig, destination, name):
    for extension in ['png', 'pdf']:
        fig.savefig(destination/f'{name}.{extension}', dpi=170, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    parser.add_argument('--allow-partial', action='store_true')
    args = parser.parse_args()
    analysis = args.batch.resolve()/'analysis'
    audit = json.loads((analysis/'audit_summary.json').read_text())
    if not args.allow_partial and not audit['complete']:
        raise ValueError('Wait for all planned runs and their audit before plotting final results')
    records = []
    for filename in sorted(analysis.glob('*_scenario*.json')):
        records.extend(json.loads(filename.read_text())['attempts'])
    for row in records:
        row['data'] = np.load(analysis/f'{row["job"]}_run{row["run"]}.npz')
    plt.rcParams.update({'font.family': FontProperties(
        fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),
        'axes.unicode_minus': False, 'font.size': 10, 'pdf.fonttype': 42})
    destination = args.batch.resolve()/'plots'
    destination.mkdir(exist_ok=True)
    mosaic = np.load(WORKSPACE/'diagnostics/isaac_observed_roads_20261025/shared_isaac_road_mosaic.npz')
    background = np.where(mosaic['road'], .93, 1.)
    lo, hi = mosaic['map_low'], mosaic['map_high']
    partial = '' if audit['complete'] else ' · 진행 중 일부 결과'
    fig, axes = plt.subplots(2, 3, figsize=(16., 10.6))
    headings, heading_axes = plt.subplots(2, 3, figsize=(16., 8.6))
    groups = []
    for number, ax, hx in zip([26, 28, 30, 27, 29, 31], axes.flat, heading_axes.flat):
        selected = [r for r in records if r['scenario']==f'scenario{number}']
        ax.imshow(background, origin='lower', extent=[lo[0], hi[0], lo[1], hi[1]],
                  cmap='gray', vmin=0., vmax=1., interpolation='nearest')
        if not selected:
            ax.set_title(f'S{number} · {LABELS[number]} · 아직 기록 없음')
            hx.set_title(f'S{number} · 아직 기록 없음')
            continue
        route = selected[0]['data']['route_xy']
        ax.plot(route[:, 0], route[:, 1], ':o', ms=3, color='#888', lw=.8)
        for r in selected:
            pose = r['data']['pose_time_xy_yaw']
            color = COLORS[r['bias_deg']]
            ax.plot(pose[:, 1], pose[:, 2], color=color, alpha=.72,
                    lw=1.5, ls=['-', '--', ':'][r['run']-1], zorder=4)
            if r['outcome'] != 'completed':
                ax.scatter(*pose[-1, 1:3], color=color, marker='x', s=55, lw=1.8, zorder=7)
            hx.plot(pose[:, 0], np.degrees(pose[:, 3]), color=color,
                    alpha=.7, lw=1.3, ls=['-', '--', ':'][r['run']-1])
        all_xy = np.vstack([route, *[r['data']['pose_time_xy_yaw'][:, 1:3] for r in selected]])
        center = (all_xy.max(axis=0)+all_xy.min(axis=0))/2.
        half = max(all_xy.max(axis=0)-all_xy.min(axis=0))/2.+12.
        ax.set(xlim=(center[0]-half, center[0]+half), ylim=(center[1]-half, center[1]+half),
               xlabel='지도 X (m)', ylabel='지도 Y (m)')
        ax.set_aspect('equal'); ax.grid(alpha=.15)
        ax.scatter(*route[0], marker='s', s=40, facecolor='white', edgecolor='#333', zorder=8)
        ax.scatter(*route[-1], marker='*', s=95, color='#333', zorder=8)
        success = sum(r['outcome']=='completed' for r in selected)
        title = f'S{number} · {LABELS[number]}\n완주 {success}/{len(selected)}회'
        ax.set_title(title, fontsize=11); hx.set_title(title, fontsize=11)
        hx.set(xlabel='주행 시작 후 시간 (초)', ylabel='차량 헤딩 (도)'); hx.grid(alpha=.2)
        for angle in COLORS:
            chosen = [r for r in selected if r['bias_deg']==angle]
            completed = [r for r in chosen if r['outcome']=='completed']
            groups.append(dict(scenario=f'scenario{number}', bias_deg=angle,
                attempts=len(chosen), completed=len(completed),
                outcomes=json.dumps(dict(Counter(r['outcome'] for r in chosen))),
                median_completed_duration_sec=float(np.median([r['duration_sec'] for r in completed])) if completed else None))
    handles = [Line2D([], [], color=c, lw=2, label=f'{a:+d}°') for a, c in COLORS.items()]
    handles.append(Line2D([], [], color='#555', marker='x', ls='', label='미완주 마지막 기록 위치'))
    for target in [fig, headings]:
        target.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .953),
                      ncol=6, frameon=False)
        target.subplots_adjust(top=.87, bottom=.10, left=.06, right=.985, wspace=.27, hspace=.34)
    fig.suptitle(f'부분 방향 바이어스 주행 자취 · {len(records)}회{partial}', fontsize=19, y=.99)
    headings.suptitle(f'부분 방향 바이어스 차량 헤딩 · {len(records)}회{partial}', fontsize=19, y=.99)
    footer = '3-state (τ=0.74초) + BnB · 색상: 경로 방향 바이어스 · 선 모양: 반복 회차 · 지도에 정렬한 EKF 위치·헤딩\n주행 구간만 표시 · 입력 끊김 회차는 기록된 부분만 표시 · 보호 정지 후 이동 제외 · 헤딩은 2π 연속화만 적용'
    fig.text(.5, .018, footer, ha='center', fontsize=10, color='#555', linespacing=1.6)
    headings.text(.5, .018, footer, ha='center', fontsize=10, color='#555', linespacing=1.6)
    save(fig, destination, 'trajectories'); save(headings, destination, 'heading')
    with (analysis/'condition_results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(groups[0]))
        writer.writeheader(); writer.writerows(groups)
    print(destination)


if __name__ == '__main__':
    main()
