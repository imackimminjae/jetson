# Closed-loop inputs for a scenario batch: accumulated static map + recorded frames + start states.
import os,sys,json,gzip,bisect,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
n=int(sys.argv[1]);out=sys.argv[2];X0,X1,Y0,Y1=map(float,sys.argv[3:7]);t_off=float(sys.argv[7])
H=f'/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario{n}/'
B={2:'20261008_183952_scenario2_5524df',3:'20261008_185426_scenario3_72c4df'}[n]
T=json.load(gzip.open(H+'telemetry.json.gz','rt'));d=T['data'];S=json.load(open(H+'summary.json'))
od=np.array([[t]+v[:6] for t,v in d['/motive/vehicle/odom_map']],dtype=float);lo=np.array([v for _,v in d['/debug/lower_mpc_trace']],dtype=float)
es=json.load(gzip.open(H+'events.json.gz','rt'));att=[(a['attempt'],a['start'],a['end']) for a in S['attempts']]
res=.25;nx,ny=int((X1-X0)/res),int((Y1-Y0)/res);occ=np.zeros((ny,nx));cnt=np.zeros((ny,nx))
g=.3125;GX,GY=np.meshgrid((np.arange(128)+.5)*g,-20+(np.arange(128)+.5)*g)
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=f'/home/imac/ros2_ws/drive_batches/{B}/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
f=open(out+'/cl_grids.bin','wb');poses=[];k=0
while r.has_next():
    _,b,tn=r.read_next();t=tn/1e9;k+=1
    if k%2: continue
    a=next((a for a in att if a[1]<=t<a[2]),None)
    if a is None: continue
    j=min(bisect.bisect_left(od[:,0],t),len(od)-1)
    if abs(od[j,0]-t)>0.06: continue
    x,y,yaw=od[j,1:4];raw=bytes(deserialize_message(b,OccupancyGrid).data);dat=np.frombuffer(raw,dtype=np.int8).reshape(128,128)
    if k%6==0:
        wx=x+np.cos(yaw)*GX-np.sin(yaw)*GY;wy=y+np.sin(yaw)*GX+np.cos(yaw)*GY;ix=((wx-X0)/res).astype(int);iy=((wy-Y0)/res).astype(int);ok=(ix>=0)&(ix<nx)&(iy>=0)&(iy<ny)&(dat>=0)
        np.add.at(occ,(iy[ok],ix[ok]),(dat[ok]>50).astype(float));np.add.at(cnt,(iy[ok],ix[ok]),1.0)
    if X0<=x<=X1 and Y0<=y<=Y1: f.write(raw);poses.append([x,y,yaw,a[0]])
f.close();json.dump(poses,open(out+'/cl_poses.json','w'))
p=np.where(cnt>0,occ/np.maximum(cnt,1),1.0);np.where(p<0.5,0,100).astype(np.int8).tofile(out+'/static_map.bin')
json.dump(dict(origin=[X0,Y0],res=res,width=nx,height=ny),open(out+'/static_map.json','w'))
starts=[]
for i,s0,s1 in att[:5]:
    t=s0+t_off;j=np.searchsorted(od[:,0],t);l=lo[np.searchsorted(lo[:,0],t)];e=min(es,key=lambda e:abs(e['t_start']-t))
    starts.append(dict(name=f's{i}',x=od[j,1],y=od[j,2],yaw=od[j,3],v=float(np.hypot(od[j,4],od[j,5])),delta=float(l[19]),wp=e['wp0']))
json.dump(starts,open(out+'/starts.json','w'));print('frames',len(poses),'starts',starts)
