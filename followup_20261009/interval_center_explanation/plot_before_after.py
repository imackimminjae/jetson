"""Compare one saved cycle; recover raw endpoints from midpoint and raw width."""
from pathlib import Path
import csv
import gzip
import json
import os

os.environ['MPLCONFIGDIR'] = '/tmp/interval_before_after_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.lines import Line2D
import numpy as np

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
HISTORY = ROOT / 'followup_20261008/batches_182344_183952_185426_review/scenario3'
events = json.load(gzip.open(HISTORY / 'events.json.gz', 'rt'))
summary = json.loads((HISTORY / 'summary.json').read_text())
attempt = next(a for a in summary['attempts'] if a['attempt'] == 2)
event = min((e for e in events if attempt['start'] <= e['t_start'] < attempt['end'] and e['y'] < 150), key=lambda e: abs(e['x'] - 251.3))
background_dir = ROOT / 'followup_20261009/s1_block_compare/regress/s3'
meta = json.loads((background_dir / 'static_map.json').read_text())
background = np.fromfile(background_dir / 'static_map.bin', dtype=np.int8).reshape(meta['height'], meta['width'])
extent = [meta['origin'][0], meta['origin'][0] + meta['width'] * meta['res'], meta['origin'][1], meta['origin'][1] + meta['height'] * meta['res']]
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})

sections = []
rows = []
for w in event['intervals_world']:
    k, candidate, x0, y0, x1, y1, raw_width, processed_width = w[:8]
    start = np.array([x0, y0])
    end = np.array([x1, y1])
    assert np.isclose(np.linalg.norm(end - start), processed_width, atol=1e-6)
    direction = (end - start) / processed_width
    midpoint = (start + end) / 2
    inset = (raw_width - processed_width) / 2
    raw_start = start - inset * direction
    raw_end = end + inset * direction
    assert np.allclose((raw_start + raw_end) / 2, midpoint)
    assert np.isclose(np.linalg.norm(raw_end - raw_start), raw_width)
    sections.append((int(k), raw_start, raw_end, start, end, midpoint, raw_width, processed_width, inset))
    rows.append(dict(k=int(k), raw_width_m=raw_width, processed_width_m=processed_width,
                     removed_each_side_m=inset, midpoint_x=midpoint[0], midpoint_y=midpoint[1],
                     reconstructed_raw_start_x=raw_start[0], reconstructed_raw_start_y=raw_start[1],
                     reconstructed_raw_end_x=raw_end[0], reconstructed_raw_end_y=raw_end[1]))

fig, axes = plt.subplots(1, 2, figsize=(13, 7.5), sharex=True, sharey=True)
for panel, ax in enumerate(axes):
    ax.imshow(np.where(background == 0, 1., .35), origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    for k, raw_start, raw_end, start, end, midpoint, raw_width, processed_width, inset in sections:
        if panel == 0:
            p, q, color, width = raw_start, raw_end, '#2776be', raw_width
        else:
            ax.plot([raw_start[0], start[0]], [raw_start[1], start[1]], '--', c='#949494', lw=2)
            ax.plot([end[0], raw_end[0]], [end[1], raw_end[1]], '--', c='#949494', lw=2)
            p, q, color, width = start, end, '#269b60', processed_width
        ax.plot([p[0], q[0]], [p[1], q[1]], c=color, lw=3, zorder=3)
        ax.plot(*midpoint, 'o', c='#202020', ms=4, zorder=4)
        label = f'k{k}: {width:.2f}m'
        if panel == 1:
            label += f'\n양끝 각각 {inset:.2f}m 제거'
        label_pos = midpoint + np.array([1.0, -.6])
        if k == 6:
            label_pos = midpoint + np.array([-5.4, 1.8])
        ax.text(*label_pos, label, fontsize=8, color=color,
                bbox=dict(facecolor='white', edgecolor='none', alpha=.8, pad=1.3), zorder=6)
    plan = np.array(event['published_world'])
    ax.plot(plan[:, 0], plan[:, 1], c='#cc173c', lw=2, zorder=5)
    ax.plot(event['x'], event['y'], 'k^', ms=8, zorder=6)
    ax.set_xlim(236, 279)
    ax.set_ylim(124, 166)
    ax.set_aspect('equal')
    ax.grid(alpha=.15)
    ax.set_xlabel('map X [m]')
    ax.set_title('축소 전 · 원래 구간' if panel == 0 else '축소 후 · 허용 구간', fontsize=13)
axes[0].set_ylabel('map Y [m]')
fig.suptitle('같은 장면의 구간 축소 전·후 — 기존 시나리오3, 2회차', fontsize=16, y=.965)
fig.legend(handles=[Line2D([], [], c='#2776be', lw=3, label='축소 전 구간'),
                    Line2D([], [], c='#269b60', lw=3, label='축소 후 구간'),
                    Line2D([], [], c='#949494', ls='--', lw=2, label='제거된 양끝'),
                    Line2D([], [], c='#202020', marker='o', ls='none', label='동일한 구간 중점'),
                    Line2D([], [], c='#cc173c', lw=2, label='같은 발행 계획')],
           loc='lower center', bbox_to_anchor=(.5, .095), ncol=5, fontsize=9)
fig.text(.045, .025,
         '축소 전 끝점은 기록된 축소 후 끝점·원래 폭과 대칭 축소 규칙으로 복원했습니다. 두 그림은 같은 시각·같은 축척입니다.\n'
         '과거 기록 soft_ratio=0.7: k2~k6은 양끝 각각 원래 폭의 30% 제거, 중앙 40% 유지. k1은 soft OFF로 양끝 각각 2m 제거.\n'
         '배경은 기존 누적 BEV 표시 자료이며, 검은 중점은 관측 구간의 중점입니다.', fontsize=8)
fig.subplots_adjust(bottom=.23, top=.87, wspace=.08)
fig.savefig(OUT / 'interval_before_after.png', dpi=180)
fig.savefig(OUT / 'interval_before_after.pdf')
plt.close(fig)
with (OUT / 'interval_before_after.csv').open('w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
(OUT / 'interval_before_after_sources.json').write_text(json.dumps(dict(
    source_events=str(HISTORY / 'events.json.gz'),
    run_id='20261008_185426_scenario3_72c4df', attempt=2, cycle_epoch=event['t_start'],
    raw_endpoints='reconstructed from recorded processed endpoints, raw widths and symmetric contraction',
    historical_soft_ratio=.7, controller_settings_changed=False,
    verified_same_midpoints=True), ensure_ascii=False, indent=2) + '\n')
print('저장 완료: interval_before_after.png / .pdf / .csv')
