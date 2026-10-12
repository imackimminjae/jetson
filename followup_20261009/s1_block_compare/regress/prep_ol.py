# Open-loop inputs for another scenario batch (read-only on bag/telemetry).
import os,sys,json,gzip,bisect,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
n=int(sys.argv[1]);out=sys.argv[2];os.makedirs(out,exist_ok=True)
H=f'/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario{n}/'
B={2:'20261008_183952_scenario2_5524df',3:'20261008_185426_scenario3_72c4df',1:'20261008_182344_scenario1_0cefb5'}[n]
T=json.load(gzip.open(H+'telemetry.json.gz','rt'));d=T['data'];S=json.load(open(H+'summary.json'))
od=np.array([[t]+v[:6] for t,v in d['/motive/vehicle/odom_map']],dtype=float);cs=np.array([v for _,v in d['/debug/upper_interval_centering_state']],dtype=float)
es=json.load(gzip.open(H+'events.json.gz','rt'));att=[(a['attempt'],a['start'],a['end']) for a in S['attempts']]
cyc=[]
for i,s0,s1 in att:
    for e in es:
        if s0<=e['t_start']<s1 and e['preview_world']:
            j=int(np.argmin(abs(cs[:,0]-e['t_start'])));oi=bisect.bisect_right(od[:,0],e['t_start'])-1
            cyc.append(dict(attempt=i,t=e['t_start'],x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(np.hypot(od[oi,4],od[oi,5])),wp0=e['wp0'],wp1=e['wp1'],preview=e['preview_world'],valid=e['valid_plan'],B_before=float(cs[j,1]),B=float(cs[j,2]),published=e['published_world']))
cyc.sort(key=lambda c:c['t'])
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=f'/home/imac/ros2_ws/drive_batches/{B}/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
ci=0;last=None;k=0;f=open(out+'/ol_grids.bin','wb')
while r.has_next():
    _,b,tn=r.read_next();t=tn/1e9
    while ci<len(cyc) and t>cyc[ci]['t']:
        f.write(last[1]);cyc[ci]['grid']=k;k+=1;ci+=1
    last=(t,bytes(deserialize_message(b,OccupancyGrid).data))
f.close();json.dump(cyc,open(out+'/ol_cycles.json','w'));json.dump(dict(route=S['route']),open(out+'/route.json','w'))
print(n,'cycles',len(cyc),'attempts',sorted(set(c['attempt'] for c in cyc)))
