from pathlib import Path
import json
import pickle
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent
root=here.parents[1]
d=pickle.load((root/'drive_debug_20261006_014143_light/analysis/decoded.pkl').open('rb'))
events=[r['data'] for r in d['/debug/upper_branch_event']]
fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True)
for ax,stamp in zip(axes[:2],['01:41:55','01:41:56']):
    e=next(e for e in events if datetime.fromtimestamp(e['t_start'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')==stamp)
    ref=np.array(e['preview_world']);plan=np.array(e['planned_world'])
    for c in e['intervals_world']:
        if c[0]!=1:continue
        a,b=np.array(c[2:4]),np.array(c[4:6]);axis=(b-a)/np.linalg.norm(b-a)
        inset=(c[6]-c[7])/2;raw=np.array([a-inset*axis,b+inset*axis]);soft=np.array([a,b])
        ax.plot(*raw.T,color='0.7',lw=8,solid_capstyle='butt')
        ax.plot(*soft.T,color='#008272',lw=5,solid_capstyle='butt')
    ax.plot(*ref[:3].T,'o--',color='#4169b1',label='Previous path at current progress')
    ax.plot(*plan[:3].T,'o-',color='#d34a36',label='Actual new plan')
    ax.scatter([e['x']],[e['y']],marker='^',s=70,c='black',zorder=5,label='Vehicle')
    ax.set_title(f'Actual S1 trace, {stamp} KST\nGray: raw section; green: allowed section')
    ax.set_aspect('equal');ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.grid(alpha=.2)
axes[0].legend(fontsize=8,loc='lower left')
cycles=json.loads((here.parent/'sc3_0141_0143/replay_S2.json').read_text())['cycles']
outs=json.loads((here/'replay_S2_out.json').read_text())
ax=axes[2]
i=next(i for i,c in enumerate(cycles) if datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')=='01:43:24')
route=np.array([[235,142],[255,135],[270,158],[278,172]])
ax.plot(*route.T,'k--o',alpha=.45,label='Global waypoints')
for key,label,color in [('0_0.000000','Current planner','#008272'),('0_0.300000','Near consistency weight 0.3','#d34a36')]:
    path=np.array(outs[key][i]['planned']);ax.plot(*path.T,'o-',color=color,label=label)
ax.set_title('S2 fixed-pose replay, 01:43:24 KST\nAdded cost keeps plan on the wrong arm')
ax.set_aspect('equal');ax.set_xlabel('World x (m)');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(here/'evidence.png',dpi=160)
fig.savefig(here/'evidence.pdf')
