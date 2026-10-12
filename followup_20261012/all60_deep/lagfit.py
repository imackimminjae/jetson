import sys,numpy as np,rosbag2_py
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261012/all60_deep')
from yawfit import score
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
TAUS=np.arange(0,0.61,0.05)
def run(bag,every,reg):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map']))
    hist=[];k=0;S=np.zeros(len(TAUS));n=0;bests=[]
    while r.has_next():
        nm,b,t=r.read_next();t=t/1e9
        if nm=='/motive/vehicle/odom_map':
            m=deserialize_message(b,Odometry);hist.append((t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)));hist=hist[-200:]
        elif len(hist)>50:
            p=hist[-1]
            if p[4]<3 or not (reg[0]<p[1]<reg[1] and reg[2]<p[2]<reg[3]): continue
            k+=1
            if k%every: continue
            d=np.frombuffer(bytes(deserialize_message(b,OccupancyGrid).data),dtype=np.int8).reshape(128,128)
            if (d<=1).sum()<500: continue
            H=np.array(hist);sc=[]
            for tau in TAUS:
                tt=t-tau;x=np.interp(tt,H[:,0],H[:,1]);y=np.interp(tt,H[:,0],H[:,2]);yw=np.interp(tt,H[:,0],np.unwrap(H[:,3]))
                sc.append(score((x,y,yw),d,0))
            sc=np.array(sc);S+=sc;n+=1;bests.append(TAUS[np.argmin(sc)])
    return S/n,np.array(bests),n
reg=(180,300,100,230)
for name,bag in [('10-08 S3','/home/imac/ros2_ws/drive_batches/20261008_185426_scenario3_72c4df/bag'),('10-09 all (S3/S10-12 region)','/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag')]:
    S,b,n=run(bag,4,reg);print(f"{name}: frames {n}; mean misfit by lag(s): "+' '.join(f"{t:.2f}:{s:.3f}" for t,s in zip(TAUS,S))+f" | best-lag median {np.median(b):.2f} s",flush=True)
