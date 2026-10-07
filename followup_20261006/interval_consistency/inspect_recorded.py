from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import pickle,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

here=Path(__file__).resolve().parent;root=here.parents[1]
data=pickle.load((root/'drive_debug_20261006_014143_light/analysis/decoded.pkl').open('rb'))
events=[x['data'] for x in data['/debug/upper_branch_event']]
grids=data['/bev/occupancy_grid'];gt=np.array([g['t'] for g in grids])
fig,axs=plt.subplots(2,3,figsize=(15,10),constrained_layout=True)
out=[]
for ax,stamp in zip(axs.flat,['01:41:53','01:41:54','01:41:55','01:41:56','01:41:58','01:41:59']):
    e=next(e for e in events if datetime.fromtimestamp(e['t_start'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')==stamp)
    t=e['t_start'];g=grids[max(0,np.searchsorted(gt,t)-1)]
    pose=np.array([e['x'],e['y']]);yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    xx,yy=np.meshgrid(np.arange(g['width']+1)*g['res']+g['origin'][0],np.arange(g['height']+1)*g['res']+g['origin'][1])
    X=xx*R[0,0]+yy*R[0,1]+pose[0];Y=xx*R[1,0]+yy*R[1,1]+pose[1]
    ax.pcolormesh(X,Y,np.where(g['grid']==0,1,np.where(g['grid']<0,.5,0)),cmap='gray',vmin=0,vmax=1,rasterized=True)
    ref=np.array(e['preview_world']);plan=np.array(e['planned_world'])
    ax.plot(*ref.T,'o--',c='royalblue',ms=3,label='previous path resampled')
    ax.plot(*plan.T,'o-',c='red',ms=3,label='new plan')
    for c in e['intervals_world']:
        if c[0] not in [1,2]:continue
        a,b=np.array(c[2:4]),np.array(c[4:6]);v=(b-a)/np.linalg.norm(b-a);ins=(c[6]-c[7])/2
        raw=np.array([a-ins*v,b+ins*v]);soft=np.array([a,b]);ax.plot(*raw.T,c='gold',lw=1.5);ax.plot(*soft.T,c='limegreen',lw=3)
    st=min(data['/debug/upper_interval_centering_state'],key=lambda a:abs(a['t']-t))['data']
    info=min(data['/debug/upper_interval_info'],key=lambda a:abs(a['t']-t))['data'].reshape(-1,12)
    row={'time':stamp,'B_before':st[1],'B':st[2],'candidate_step':int(st[5]),'intervals':info[info[:,0]<=2].tolist()}
    print(stamp,'B',st[1:3], 'selected',e['selected_corridors'][:3], 'info',info[(info[:,0]>0)&(info[:,0]<=2),:8].tolist())
    out.append(row);ax.set_title(f'{stamp} B={st[1]:.1f} -> {st[2]:.1f}')
    ax.set_xlim(pose[0]-6,pose[0]+19);ax.set_ylim(pose[1]-21,pose[1]+4);ax.set_aspect('equal');ax.grid(alpha=.1)
axs[0,0].legend(fontsize=7)
fig.savefig(here/'recorded_sections.png',dpi=150)
(here/'recorded_sections.json').write_text(json.dumps(out,indent=2)+'\n')
