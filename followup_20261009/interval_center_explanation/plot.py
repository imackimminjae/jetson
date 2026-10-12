"""Show recorded processed interval midpoints, without modifying planner settings."""
from pathlib import Path
import os,gzip,json,csv
os.environ['MPLCONFIGDIR']='/tmp/interval_center_explanation_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.lines import Line2D
R=Path('/home/imac/ros2_ws');O=Path(__file__).resolve().parent
H=R/'followup_20261008/batches_182344_183952_185426_review/scenario3'
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
es=json.load(gzip.open(H/'events.json.gz','rt'));s=json.loads((H/'summary.json').read_text())
b=R/'followup_20261009/s1_block_compare/regress/s3'
m=json.loads((b/'static_map.json').read_text())
bg=np.fromfile(b/'static_map.bin',dtype=np.int8).reshape(m['height'],m['width'])
ext=[m['origin'][0],m['origin'][0]+m['width']*m['res'],m['origin'][1],m['origin'][1]+m['height']*m['res']]
fig,axs=plt.subplots(1,2,figsize=(12,6.5));rows=[]
for ax,(i,x) in zip(axs,[(2,251.3),(4,253.7)]):
    a=next(a for a in s['attempts'] if a['attempt']==i)
    e=min((e for e in es if a['start']<=e['t_start']<a['end'] and e['y']<150),key=lambda e:abs(e['x']-x))
    ax.imshow(np.where(bg==0,1.,.35),origin='lower',extent=ext,cmap='gray',vmin=0,vmax=1)
    mids=[]
    for w in e['intervals_world']:
        k,c,x0,y0,x1,y1,raw,processed=w[:8];mid=np.array([(x0+x1)/2,(y0+y1)/2]);mids.append(mid)
        ax.plot([x0,x1],[y0,y1],c='#269b60',lw=3);ax.plot(*mid,'o',c='#0066cc',ms=4)
        ax.text(mid[0]+(.7 if k<6 else -4),mid[1]-.5,f'k{int(k)} · {processed:.2f}m',fontsize=8,c='#0066cc')
        p=e['published_world'][int(k)] if e['published_world'] and int(k)<len(e['published_world']) else None
        direction=np.array([x1-x0,y1-y0])/processed
        u=float(np.dot(np.array(p)-np.array([x0,y0]),direction)) if p else None
        rows.append(dict(attempt=i,cycle_epoch=e['t_start'],k=int(k),raw_width_m=raw,processed_width_m=processed,mid_x=mid[0],mid_y=mid[1],plan_to_processed_start_m=u,plan_to_processed_end_m=processed-u if p else None,plan_center_lateral_offset_m=u-processed/2 if p else None))
    mids=np.array(mids);ax.plot(mids[:,0],mids[:,1],'--',c='#0066cc',lw=1.2)
    pv=np.array(e['preview_world']);ax.plot(pv[:,0],pv[:,1],':',c='#777',lw=1)
    if e['published_world']:
        p=np.array(e['published_world']);ax.plot(p[:,0],p[:,1],c='#cc173c',lw=2)
        ax.annotate('k5 횡방향 여유\n0.34m / 3.14m',p[5],xytext=(239,132),fontsize=9,c='#b51c45',bbox=dict(fc='white',alpha=.9),arrowprops=dict(arrowstyle='->',color='#b51c45'))
    ax.plot(e['x'],e['y'],'k^',ms=7);ax.set_xlim(238,275);ax.set_ylim(128,162);ax.set_aspect('equal');ax.grid(alpha=.18)
    ax.set_title(f'기존 시나리오3 A{i} · 해당 주기 {"계획 발행" if e["valid_plan"] else "QP 미해결"}',fontsize=11)
    ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
fig.suptitle('허용 구간 중점과 발행 계획의 위치 비교',fontsize=16,y=.97)
fig.legend(handles=[Line2D([],[],c='#269b60',lw=3,label='처리 후 허용 구간'),Line2D([],[],c='#0066cc',ls='--',marker='o',label='구간 중점 연결'),Line2D([],[],c='#cc173c',lw=2,label='발행 계획'),Line2D([],[],c='#777',ls=':',label='기존 참조')],ncol=4,loc='lower center',bbox_to_anchor=(.5,.075),fontsize=9)
fig.text(.035,.025,'선 옆 숫자: 처리 후 길이 | 파란 중점은 관측 구간의 중점이며 독립 도로 중심선 측정이 아님\n과거 기록 soft_ratio=0.7 (중앙 40% 유지); k1은 양끝2m 제거(soft OFF). 배경은 기존 누적 BEV 표시 자료.',fontsize=8)
fig.subplots_adjust(bottom=.20,top=.85,wspace=.25)
fig.savefig(O/'interval_midpoints_vs_plan.png',dpi=160);fig.savefig(O/'interval_midpoints_vs_plan.pdf');plt.close(fig)
with (O/'stage_offsets.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
print('중점 비교 그림 저장 완료')
