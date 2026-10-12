import sys,json,numpy as np,rosbag2_py,collections
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261012/all60_deep')
from yawfit import score
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
TAUS=np.arange(0,0.61,0.05)
def run(bag,every):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map','/knu_batch/active_route']))
    hist=[];k=0;acc=collections.defaultdict(lambda:[np.zeros(len(TAUS)),0]);route='?';gts=collections.defaultdict(list)
    while r.has_next():
        nm,b,t=r.read_next();t=t/1e9
        if nm=='/knu_batch/active_route':
            try: route=json.loads(deserialize_message(b,String).data).get('route',route)
            except Exception: pass
        elif nm=='/motive/vehicle/odom_map':
            m=deserialize_message(b,Odometry);hist.append((t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)));hist=hist[-200:]
        else:
            gts[route].append(t)
            if len(hist)<50 or hist[-1][4]<3: continue
            k+=1
            if k%every: continue
            d=np.frombuffer(bytes(deserialize_message(b,OccupancyGrid).data),dtype=np.int8).reshape(128,128)
            if (d<=1).sum()<500: continue
            H=np.array(hist);sc=[]
            for tau in TAUS:
                tt=t-tau;sc.append(score((np.interp(tt,H[:,0],H[:,1]),np.interp(tt,H[:,0],H[:,2]),np.interp(tt,H[:,0],np.unwrap(H[:,3]))),d,0))
            acc[route][0]+=np.array(sc);acc[route][1]+=1
    return acc,gts
import sys as _s
for bag in _s.argv[1:]:
    acc,gts=run(bag,int(8 if 'all' in bag else 3))
    print('==',bag.split('/')[-2])
    for rt,(S,n) in sorted(acc.items(),key=lambda kv:int(kv[0][8:]) if kv[0].startswith('scenario') else 0):
        g=np.diff(np.array(gts[rt]));g=g[g<0.5]
        print(f"   {rt:11s} frames {n:4d} best lag {TAUS[np.argmin(S/n)]:.2f} s  BEV rate {1/np.median(g):.0f} Hz")
