from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
H=Path(__file__).resolve().parent
fig,axs=plt.subplots(1,2,figsize=(12,5.2),layout='constrained')
rows=json.loads((H/'k2_reachable.json').read_text());ax=axs[0]
for i,r in enumerate(rows):
 a,b=r['k2_required_correction'];lo,hi=r['k2_reachable_correction']
 ax.plot([lo,hi],[i+.12]*2,lw=8,color='#4897cc',solid_capstyle='butt',label='Reachable under input limits' if i==0 else None)
 ax.plot([a,b],[i-.12]*2,lw=8,color='#d7544b',solid_capstyle='butt',label='Required by k2 road interval' if i==0 else None)
 gap=max(a-hi,lo-b,0)
 ax.text(14,i,f'gap {gap:.3f} m',va='center',fontsize=9)
ax.set_yticks(range(3),['18:49:23.269','18:50:44.382','18:52:12.630'])
ax.set_xlabel('Lateral correction at k2 relative to reference (m)')
ax.set_title('k1 removed: k2 still unreachable\nConditional-skip carried replay, first failures')
ax.set_xlim(-15,20);ax.set_ylim(-.7,3);ax.grid(axis='x',alpha=.2);ax.legend(loc='upper left',fontsize=8)
tag='185025';i=32;p=H.parent/'origin295_drive_review'
c=json.loads((p/f'input_{tag}.json').read_text())['cycles'][i]
r=json.loads((H/f'result_{tag}.json').read_text())['carried_m1'][i]
g=np.fromfile(p/f'grids_{tag}.bin',dtype=np.int8).reshape(-1,128,128)[c['grid_index']]
yaw=c['yaw'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
def body(q):return (np.array(q)-np.array([c['x'],c['y']]))@R
ax=axs[1]
ax.set_facecolor('#dde1e6')
ax.imshow(g,extent=[2.95,42.95,-20,20],origin='lower',cmap=ListedColormap(['white','#444444']),norm=BoundaryNorm([-.5,.5,100.5],2),interpolation='nearest')
ref=body(r['reference']);plan=body(r['planned'])
ax.plot(ref[:,0],ref[:,1],'--',c='#aa73c8',label='Unchanged reference')
ax.plot(plan[:,0],plan[:,1],'-o',c='#198dd0',ms=4,label='Plan after k1 skip')
ab=body(np.array(r['cands'][1][0]).reshape(2,2));ax.plot(ab[:,0],ab[:,1],c='#e6a22f',lw=4,label='Removed k1 interval')
ax.axvline(2.95,ls=':',c='black');ax.scatter(*plan[1],s=130,facecolors='none',edgecolors='red',lw=2,zorder=10)
ax.annotate('k1 = (4.46, -2.86) m\nCell value = 100',plan[1],xytext=(5.5,-5.8),arrowprops=dict(arrowstyle='->',color='red'),fontsize=9,color='#ba2222')
ax.plot(0,0,'k^',ms=8);ax.text(.2,3.5,'Unobserved\nx < 2.95 m',fontsize=9)
ax.set_xlim(-.5,12);ax.set_ylim(-7,5);ax.set_aspect('equal');ax.set_xlabel('Vehicle forward x (m)');ax.set_ylabel('Vehicle left y (m)')
ax.set_title('18:50:44.881: newly feasible, but k1 in blocked cell')
ax.legend(loc='upper right',fontsize=8)
fig.savefig(H/'k1_skip_limits.png',dpi=170)
