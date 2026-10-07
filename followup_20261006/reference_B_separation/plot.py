from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
here=Path(__file__).resolve().parent;D=json.loads((here/'swap_result.json').read_text());K=json.loads((here/'swap_keys.json').read_text());C=json.loads((here/'swap_input.json').read_text())['cycles']
fig,axs=plt.subplots(2,2,figsize=(11,10),sharex=True,sharey=True)
for i,k in enumerate(K):
 if k['time']!='05:06:51.906':continue
 ri=int(k['reference_source']==12);bi=int(k['B_source']==12);ax=axs[ri,bi];r=D['cap0'][i];c=C[i]
 grid=np.fromfile(here/'replay_050627_grids.bin',np.int8).reshape(-1,128,128)[c['grid_index']]
 xx,yy=np.meshgrid(np.arange(129)*.3125,np.arange(129)*.3125-20);yaw=c['yaw'];X=c['x']+np.cos(yaw)*xx-np.sin(yaw)*yy;Y=c['y']+np.sin(yaw)*xx+np.cos(yaw)*yy
 ax.pcolormesh(X,Y,grid,cmap='Greys',vmin=0,vmax=100,alpha=.25,rasterized=True)
 for step in [5,6,7]:
  for q in r['cands'][step]:
   q=np.array(q).reshape(2,2);ax.plot(q[:,0],q[:,1],color=('#d95f02' if step==7 else '#666666'),lw=4 if step==7 else 2)
  q=np.array(r['cands'][step][0]).reshape(2,2).mean(axis=0);ax.text(q[0]+.3,q[1],f'k{step}',fontsize=9)
 ref=np.array(r['reference']);p=np.array(r['planned']);ax.plot(ref[:,0],ref[:,1],'--',color='#7b3294',label='Reference');ax.plot(p[:,0],p[:,1],'-o',color='#0072b2',ms=3,label='New plan');ax.scatter(c['x'],c['y'],marker='^',color='black',label='Recorded pose')
 direction='EAST' if ri==0 else 'NORTH'
 ax.set_title(f'{"Baseline" if ri==0 else "Cap-history"} reference | B={r["B"]:.1f} m\nPlan: {direction}; turn window [3,4,5]')
 ax.set_xlim(109,146);ax.set_ylim(-23,22);ax.set_aspect('equal');ax.grid(alpha=.15)
axs[0,0].legend(fontsize=8,loc='upper right');fig.supxlabel('World x (m)');fig.supylabel('World y (m)')
fig.suptitle('05:06:51.906 — reference × updated B crossover\nSame recorded pose / BEV; soft cap disabled in all four instantaneous solves',fontsize=13)
fig.tight_layout();fig.savefig(here/'reference_B_crossover.png',dpi=170);plt.close(fig)
