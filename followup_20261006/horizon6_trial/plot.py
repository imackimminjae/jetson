from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path(__file__).resolve().parent;old=h.parent/'reference_B_separation'
fig,axs=plt.subplots(1,3,figsize=(14,5))
cases=[('050627',1791230811.906,'05:06:51.906 — scenario1'),('014303',None,'01:43:24.894 — scenario3'),('055105',None,'05:51:30.837 — scenario1')]
from datetime import datetime
from zoneinfo import ZoneInfo
for ax,(tag,_,title) in zip(axs,cases):
 cs=json.loads((old/f'replay_{tag}.json').read_text())['cycles'];stamp=title.split(' —')[0];i=next(i for i,c in enumerate(cs) if datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f').startswith(stamp));c=cs[i]
 grid=np.fromfile(old/f'replay_{tag}_grids.bin',np.int8).reshape(-1,128,128)[c['grid_index']];xx,yy=np.meshgrid(np.arange(129)*.3125,np.arange(129)*.3125-20);yaw=c['yaw'];X=c['x']+np.cos(yaw)*xx-np.sin(yaw)*yy;Y=c['y']+np.sin(yaw)*xx+np.cos(yaw)*yy;ax.pcolormesh(X,Y,grid,cmap='Greys',vmin=0,vmax=100,alpha=.2,rasterized=True)
 for n,color in [(7,'#d55e00'),(6,'#0072b2')]:
  r=json.loads((h/f'result_{tag}_current_h{n}_recorded.json').read_text())['carried'][i];p=np.array(r['planned']);ax.plot(p[:,0],p[:,1],'-o',color=color,ms=3,lw=2,label=f'Max {n} stages')
 ax.scatter(c['x'],c['y'],marker='^',color='black',label='Recorded pose');ax.set_title(title);ax.grid(alpha=.15);ax.set_aspect('equal');ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)')
 if tag in ['050627','055105']:ax.set_xlim(109,145);ax.set_ylim(-23,17)
 else:ax.set_xlim(242,282);ax.set_ylim(127,166)
axs[0].legend(fontsize=8);fig.suptitle('Horizon 7 → 6, current parameter snapshot; all other parameters fixed\nRecorded pose / BEV replay, with each variant carrying its own plan and B',fontsize=12);fig.tight_layout();fig.savefig(h/'horizon6_replay.png',dpi=160)
