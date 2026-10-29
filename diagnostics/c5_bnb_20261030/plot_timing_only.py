"""Plot matched-input MIQP solve times only."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/c5_solver_plot')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
from pathlib import Path
import json
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

out = Path(__file__).resolve().parent
data = json.loads((out / 'replay_common_valid_summary.json').read_text())
counts = json.loads((out / 'replay_binary_counts.json').read_text())
with (out / 'replay_final.csv').open() as stream:
    rows = list(csv.DictReader(stream))
valid_frames = {
    mode: {r['frame'] for r in rows if r['mode'] == mode and r['valid'] == '1'}
    for mode in ('hybrid', 'bnb')}
common_frames = valid_frames['hybrid'] & valid_frames['bnb']
max_cases = {}
plt.rcParams.update({
    'font.family': FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),
    'axes.unicode_minus': False, 'font.size': 12,
    'axes.spines.top': False, 'axes.spines.right': False, 'pdf.fonttype': 42})
fig, ax = plt.subplots(figsize=(9.2, 5.4))
for index, (mode, label, color) in enumerate([
        ('hybrid', '기존 Hybrid', '#D88137'), ('bnb', '전 구간 BnB', '#1878B5')]):
    values = [data[mode]['miqp_solver_ms'][key] for key in ('mean', 'p95', 'max')]
    x = np.arange(3) + (-.18 if index == 0 else .18)
    ax.bar(x, values, width=.32, color=color, label=label, zorder=3)
    for xx, value in zip(x, values):
        ax.text(xx, value + .18, f'{value:.2f} ms', ha='center', color=color, fontsize=12)
    max_row = max((r for r in rows if r['mode'] == mode and r['valid'] == '1'
                   and r['frame'] in common_frames), key=lambda r: float(r['miqp_solver_ms']))
    assert np.isclose(float(max_row['miqp_solver_ms']), values[-1])
    case = counts[max_row['frame']]
    max_cases[mode] = dict(frame=int(max_row['frame']), repeat=int(max_row['repeat']),
                          miqp_solver_ms=values[-1], **case)
    ax.text(x[-1], values[-1] + 1.7, f"이진변수 {case['binary_count']}개",
            ha='center', color=color, fontsize=11,
            bbox=dict(boxstyle='round,pad=.35', fc='white', ec=color, alpha=.95))
ax.set(xticks=[0, 1, 2], xticklabels=['평균', '95백분위', '최대'],
       ylabel='연산시간 (ms)', ylim=(0, 17.8))
ax.grid(axis='y', alpha=.2)
ax.set_axisbelow(True)
ax.legend(frameon=False, ncol=2, loc='upper left')
fig.suptitle('MIQP 연산시간 비교', fontsize=20, y=.97)
fig.text(.5, .085, '두 방식 모두 성공한 동일 입력 344개 / 전체 351개 · 입력당 방식별 2회',
         ha='center', color='#555', fontsize=10)
fig.text(.5, .04, '정수 선택 후 연속 QP 보정 포함 · 후속 물리 제약 QP 및 경로 생성 시간 제외',
         ha='center', color='#555', fontsize=10)
(out / 'timing_max_cases.json').write_text(json.dumps(dict(
    binary_count_definition='Sum of corridor candidate counts over prediction stages, as in binary_offsets.back()',
    note='Each method has its own maximum-time input; count refers to the original MIQP formulation.',
    cases=max_cases), ensure_ascii=False, indent=2) + '\n')
fig.subplots_adjust(left=.10, right=.98, top=.85, bottom=.22)
for extension in ('png', 'pdf'):
    fig.savefig(out / f'timing_only.{extension}', dpi=190, facecolor='white')
print(out / 'timing_only.png')
