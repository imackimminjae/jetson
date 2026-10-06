from pathlib import Path
import pickle,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from datetime import datetime
p=Path(__file__).resolve().parent;d=pickle.load(open(p/'decoded.pkl','rb'))
events=[x['data'] for x in d['/debug/upper_branch_event'] if x['data']['reason']=='solver_rejected']
fig,axs=plt.subplots(1,3,figsize=(16,7))
for ax,e in zip(axs,[events[0],events[2],events[3]]):
 g=max((x for x in d['/bev/occupancy_grid'] if x['t']<=e['t_start']),key=lambda x:x['t'])
 yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xy=np.array([e['x'],e['y']])
 transform=lambda a:(np.array(a)-xy)@R
 grid=np.where(g['grid']<0,0,np.where(g['grid']==0,1,2));ox,oy=g['origin'];res=g['res']
 ax.imshow(grid,origin='lower',extent=(ox,ox+g['width']*res,oy,oy+g['height']*res),cmap=ListedColormap(['#989898','#f4f4f4','#343434']),vmin=0,vmax=2,interpolation='nearest')
 for row in e['intervals_world']:
  if int(row[0])>4:continue
  ends=transform(np.array(row[2:6]).reshape(2,2));mid=ends.mean(axis=0);dire=ends[1]-ends[0];dire/=np.linalg.norm(dire)
  raw=np.array([mid-dire*row[6]/2,mid+dire*row[6]/2]);ax.plot(*raw.T,color='#e8a317',lw=2,alpha=.8)
  ax.plot(*ends.T,color='#16acd2',lw=4)
 ref=transform(e['preview_world']);ax.plot(*ref.T,'o--',color='#e44963',ms=3,lw=1,label='Planning reference')
 old=transform(e['retained_world'])
 if len(old):ax.plot(*old.T,color='#70e67a',lw=1.8,label='Retained old plan')
 tr=min(d['/debug/upper_miqp_trace'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data'];h=tr[18];axis=np.array([-np.sin(h),np.cos(h)])
 ax.plot(*np.array([-15*axis,15*axis]).T,':',color='#c269ed',label='k=0 cross-section')
 ax.scatter([0],[0],c='#ef3b2c',s=45,zorder=10);ax.arrow(0,0,2,0,width=.07,color='#ef3b2c',length_includes_head=True,zorder=10)
 ax.axvspan(-3,0,facecolor='#b6b6b6',hatch='//',alpha=.6,zorder=.5)
 ax.axvline(0,color='#999999',ls='--',lw=.7)
 ax.set(xlim=(-3,24),ylim=(-13,13),aspect='equal',xlabel='Vehicle forward (m)',ylabel='Vehicle left (m)',title=datetime.fromtimestamp(e['t_start']).strftime('%H:%M:%S')+'  QP infeasible')
 ax.grid(alpha=.1)
axs[0].legend(loc='lower right',fontsize=8)
fig.suptitle('Latest failures: white=observed road, dark=blocked, gray=unknown/outside\nOrange=raw observed intervals; cyan=constrained intervals; red origin=current vehicle',fontsize=12)
fig.tight_layout();fig.savefig(p/'failure_grid_comparison.png',dpi=155)
