"""Aggregate recorded BEV grids (receipt-time pose alignment) into a map-frame occupancy world for scenario1."""
import pickle, numpy as np, json
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
X0,Y0,RES,W,H=50.0,-160.0,0.25,600,1000
free=np.zeros((H,W),np.int32); blk=np.zeros((H,W),np.int32)
for p in ['/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis/decoded.pkl']:
    d=pickle.load(open(p,'rb'))
    od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); oyaw=np.unwrap([x['q'][2] for x in od])
    for g in d['/bev/occupancy_grid'][::3]:
        if g['t']<ot[0] or g['t']>ot[-1]: continue
        pos=np.array([np.interp(g['t'],ot,oxy[:,0]),np.interp(g['t'],ot,oxy[:,1])]); yaw=np.interp(g['t'],ot,oyaw)
        grid=g['grid']; ij=np.argwhere(grid>=0); v=grid[ij[:,0],ij[:,1]]
        body=np.array(g['origin'])+(ij[:,::-1]+.5)*g['res']; c,s=np.cos(yaw),np.sin(yaw)
        wx=c*body[:,0]-s*body[:,1]+pos[0]; wy=s*body[:,0]+c*body[:,1]+pos[1]
        ix=((wx-X0)/RES).astype(int); iy=((wy-Y0)/RES).astype(int); m=(ix>=0)&(ix<W)&(iy>=0)&(iy<H)
        np.add.at(free,(iy[m&(v==0)],ix[m&(v==0)]),1); np.add.at(blk,(iy[m&(v>0)],ix[m&(v>0)]),1)
world=np.full((H,W),-1,np.int8); seen=(free+blk)>0
world[seen&(free>blk)]=0; world[seen&(free<=blk)]=100
# fill 1-cell holes between projected samples (grid res 0.3125 > world res 0.25)
from scipy.ndimage import binary_closing
fr=binary_closing(world==0,iterations=1); world[(world==-1)&fr]=0
world.tofile('recorded_world.bin')
json.dump({'origin':[X0,Y0],'resolution':RES,'width':W,'height':H,'free_cells':int((world==0).sum()),'blocked':int((world==100).sum()),'unknown':int((world==-1).sum())},open('recorded_world_meta.json','w'))
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
plt.figure(figsize=(9,14)); img=np.where(world==0,1.0,np.where(world==100,0.0,0.6))
plt.imshow(img,origin='lower',extent=[X0,X0+W*RES,Y0,Y0+H*RES],cmap='gray',vmin=0,vmax=1); plt.plot(GP[:,0],GP[:,1],'r--o'); plt.title('aggregated recorded BEV world (white=free, gray=unknown)'); plt.savefig('recorded_world.png',dpi=80)
print(open('recorded_world_meta.json').read())
