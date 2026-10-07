import numpy as np, pickle, time
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from analyze_runs import ROUTES, clock
d=pickle.load(open('/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis/decoded.pkl','rb'))
SER=pickle.load(open('../runs_series.pkl','rb'))
od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); oyaw=np.unwrap([x['q'][2] for x in od])
gr=d['/bev/occupancy_grid']; gt=np.array([x['t'] for x in gr])
ev=d['/debug/upper_branch_event']
GP=ROUTES['scenario1']
times=['23:45:28','23:45:30','23:45:32','23:45:34','23:45:36','23:45:38']
fig,axs=plt.subplots(2,3,figsize=(21,14))
for ax,tsr in zip(axs.flat,times):
    t=time.mktime(time.strptime('2026-10-05 '+tsr,'%Y-%m-%d %H:%M:%S'))
    i=np.searchsorted(gt,t)-1; g=gr[i]; pos=np.array([np.interp(g['t'],ot,oxy[:,0]),np.interp(g['t'],ot,oxy[:,1])]); yaw=np.interp(g['t'],ot,oyaw)
    grid=g['grid']; ij=np.argwhere(grid>=-1); body=np.array(g['origin'])+(ij[:,::-1]+.5)*g['res']
    c,s=np.cos(yaw),np.sin(yaw); w=np.stack([c*body[:,0]-s*body[:,1]+pos[0],s*body[:,0]+c*body[:,1]+pos[1]],1); v=grid[ij[:,0],ij[:,1]]
    ax.scatter(w[:,0],w[:,1],c=np.where(v==0,'#9be39b',np.where(v==100,'#444444','#dddddd')),s=3,marker='s',lw=0)
    ax.plot(GP[:,0],GP[:,1],'k--o',ms=4,label='global path')
    for k,col in (('A_1003_0559_tau005','red'),('R3_1005_2345_sc1','blue')):
        S=SER[k]; ax.plot(S['mx'],S['my'],color=col,lw=1,label=k)
    e=min(ev,key=lambda x:abs(x['t']-t))['data']
    if e['valid_plan']: P=np.array(e['planned_world']); ax.plot(P[:,0],P[:,1],'m.-',label='upper plan')
    R=np.array(e['preview_world']); ax.plot(R[:,0],R[:,1],'c^-',ms=4,label='preview reference')
    ax.plot(pos[0],pos[1],'r*',ms=14); ax.set_xlim(pos[0]-40,pos[0]+45); ax.set_ylim(pos[1]-30,pos[1]+50); ax.set_aspect('equal'); ax.grid(alpha=.3)
    ax.set_title(f'R3 grid {clock(g["t"])}  wp={e["wp0"]},{e["wp1"]}'); ax.legend(fontsize=7)
plt.tight_layout(); plt.savefig('../figures/junctions_R3.png',dpi=75); print('ok')
print({k:v for k,v in ev[12]['data'].items() if not isinstance(v,list)})
