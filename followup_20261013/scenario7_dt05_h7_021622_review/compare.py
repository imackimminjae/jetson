"""Compare already extracted previous analysis with latest local signals."""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import os
os.environ['MPLCONFIGDIR'] = '/tmp/s7_dt05_compare_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from PIL import Image

OUT = Path(__file__).resolve().parent
OLD = OUT.parent/'scenarios_7_8_12_review'
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})
colors = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader(
    (Path('/home/imac/ros2_ws/drive_batches/20261013_021622_scenario7_09689d/routes/scenario7.csv')).open())])
fig, axes = plt.subplots(1, 2, figsize=(12, 8))
for ax, folder, title in zip(axes, [OLD, OUT], ['이전: dt0.75 · 6단계 · 약27m · 목표 접근0/5',
                                                        '이번: dt0.5 · 7단계 · 약21m · 목표 접근4/5']):
    data = np.load(folder/'scenario7/signals.npz')['odom']
    rows = list(csv.DictReader((folder/'scenario7/per_attempt.csv').open()))
    ax.plot(route[:,0], route[:,1], 'k--o', lw=1.2, ms=4, label='waypoint')
    ax.plot(*route[-1], 'k*', ms=11)
    for a, color in zip(rows, colors):
        start, end, stop = map(float, [a['start_epoch_s'], a['outcome_epoch_s'], a['stop_phase_epoch_s']])
        tr = data[(data[:,0]>=start)&(data[:,0]<stop)]
        run = tr[:,0]<end
        outcome = '목표 접근' if a['outcome']=='completed' else 'off_route'
        ax.plot(tr[run,1],tr[run,2],color=color,lw=1.5,label=f'{a["attempt"]}회 {outcome}')
        ax.plot(tr[~run,1],tr[~run,2],':',color=color,lw=1)
        ax.plot(float(a['stop_x']),float(a['stop_y']),'s',mfc='white',color=color,ms=4)
    ax.set(xlim=(-135,-40),ylim=(0,135),xlabel='map X [m]',ylabel='map Y [m]',title=title)
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.suptitle('시나리오7 · 설정 변경 전후5회 비교\nBEV40×40m 유지 · 실선 running · 점선 관성 이동 · □ 정지 · ★ 목표',fontsize=13)
fig.tight_layout(rect=(0,0,1,.92))
fig.savefig(OUT/'scenario7_before_after.png',dpi=170)
fig.savefig(OUT/'scenario7_before_after.pdf')
plt.close(fig)

v=json.loads((OUT/'validation.json').read_text())
for name,expected in json.loads((OUT/'sources_sha256.json').read_text()).items():
    h=hashlib.sha256()
    with Path(name).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    assert h.hexdigest()==expected,name
for path in OUT.glob('*.png'):
    with Image.open(path) as im:im.verify()
selected=json.loads((OUT/'selected_branch_cycles.json').read_text())
assert len(selected)==2
v.update(grid_decoded=2,selected_grid_frames=2,grid_scope='two selected frames only; no accumulated map cache',
         all_original_hashes_rechecked=True,previous_comparison_source='already extracted earlier analysis, not a new replay',
         selected_frame_age_max_ms=max(c['grid_before_plan_ms'] for c in selected))
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('전후 비교 그림 저장, 원본 해시와 PNG 검증 완료')
