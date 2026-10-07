import numpy as np, pickle, time
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
d = pickle.load(open('/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'))
res = pickle.load(open('traj_res.pkl','rb'))
od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); oyaw=np.unwrap([x['q'][2] for x in od])
gr=d['/bev/occupancy_grid']; gt=np.array([x['t'] for x in gr])
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
fig,axs=plt.subplots(2,3,figsize=(21,14))
times=['05:59:30','05:59:34','05:59:37','05:59:40','05:59:44','05:59:48']
for ax,tsr in zip(axs.flat,times):
    t=time.mktime(time.strptime('2026-10-03 '+tsr,'%Y-%m-%d %H:%M:%S'))
    i=np.searchsorted(gt,t)-1; g=gr[i]; pos=np.array([np.interp(g['t'],ot,oxy[:,0]),np.interp(g['t'],ot,oxy[:,1])]); yaw=np.interp(g['t'],ot,oyaw)
    grid=g['grid']; H,W=grid.shape; ij=np.argwhere(grid>=-1)
    body=np.array(g['origin'])+(ij[:,::-1]+0.5)*g['res']   # (x,y) body
    c,s=np.cos(yaw),np.sin(yaw); world=np.stack([c*body[:,0]-s*body[:,1]+pos[0], s*body[:,0]+c*body[:,1]+pos[1]],1)
    vals=grid[ij[:,0],ij[:,1]]
    col=np.where(vals==0,'#9be39b',np.where(vals==100,'#444444','#dddddd'))
    ax.scatter(world[:,0],world[:,1],c=col,s=3,marker='s',lw=0)
    ax.plot(GP[:,0],GP[:,1],'k--o',ms=3)
    for k,cc in (('A','red'),('B','blue')):
        R=res[k]; ax.plot(R['x'],R['y'],color=cc,lw=1.2,label=f'run {k}')
    ax.plot(pos[0],pos[1],'r*',ms=12); ax.set_title(f'A grid received {clock(g["t"])} (pose-aligned at receipt)'); ax.set_aspect('equal'); ax.set_xlim(pos[0]-45,pos[0]+45); ax.set_ylim(pos[1]-45,pos[1]+45); ax.grid(True,alpha=.3); ax.legend(fontsize=8)
plt.tight_layout(); plt.savefig('grids_branch_AB.png',dpi=80); print('ok')
