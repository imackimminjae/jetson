import json,numpy as np,rosbag2_py,collections,sys
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261012/all60_deep')
from yawfit import score
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
def run(bag,every,region=None):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map','/knu_batch/active_route']))
    pose=None;k=0;res=[];route='?'
    while r.has_next():
        n,b,t=r.read_next()
        if n=='/knu_batch/active_route':
            try: route=json.loads(deserialize_message(b,String).data).get('route',route)
            except Exception: pass
        elif n=='/motive/vehicle/odom_map':
            m=deserialize_message(b,Odometry);pose=(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
        elif pose is not None and pose[3]>2:
            if region and not (region[0]<pose[0]<region[1] and region[2]<pose[1]<region[3]): continue
            k+=1
            if k%every: continue
            d=np.frombuffer(bytes(deserialize_message(b,OccupancyGrid).data),dtype=np.int8).reshape(128,128)
            if (d<=1).sum()<500: continue
            best=min(((score(pose,d,0,dx,dy),dx,dy) for dx in np.arange(-4,4.1,0.5) for dy in np.arange(-4,4.1,0.5)))
            # express shift in vehicle frame (forward, left)
            c,s=np.cos(pose[2]),np.sin(pose[2]);fwd=best[1]*c+best[2]*s;left=-best[1]*s+best[2]*c
            res.append((route,pose[0],pose[1],best[1],best[2],fwd,left,best[0]))
    return res
reg=(180,270,125,225)
for name,bag in [('10-08 S3','/home/imac/ros2_ws/drive_batches/20261008_185426_scenario3_72c4df/bag'),('10-09 all (S3 region)','/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag')]:
    R=run(bag,6,reg);a=np.array([x[3:7] for x in R])
    print(f"{name}: frames {len(R)} world shift median dx {np.median(a[:,0]):+.1f} dy {np.median(a[:,1]):+.1f} | vehicle-frame median fwd {np.median(a[:,2]):+.1f} left {np.median(a[:,3]):+.1f}",flush=True)
    if 'all' in name:
        by=collections.defaultdict(list)
        for x in R: by[x[0]].append(x[3:7])
        for k,v in by.items(): v=np.array(v);print(f"    {k}: n={len(v)} dx {np.median(v[:,0]):+.1f} dy {np.median(v[:,1]):+.1f} fwd {np.median(v[:,2]):+.1f} left {np.median(v[:,3]):+.1f}")
