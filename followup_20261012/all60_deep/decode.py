# Decode the full 60-run batch once into a compact pickle (no grids except sparse frames for map accumulation).
import os,json,pickle,collections,numpy as np,rosbag2_py,bisect
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
B='/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag'
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=B,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:get_message(x.type) for x in r.get_all_topics_and_types()}
keep={'/knu_batch/status','/debug/upper_branch_event','/debug/upper_interval_centering_state','/motive/vehicle/odom_map','/debug/lower_mpc_trace','/debug/upper_interval_info','/rosout','/knu_batch/active_route','/debug/upper_stage_timing'}
D=collections.defaultdict(list);grids=[];gk=0
while r.has_next():
    n,b,tn=r.read_next();t=tn/1e9
    if n=='/bev/occupancy_grid':
        gk+=1
        if gk%4: continue
        m=deserialize_message(b,types[n]);grids.append((t,bytes(m.data)));continue
    if n not in keep: continue
    m=deserialize_message(b,types[n])
    if n in ('/knu_batch/status','/debug/upper_branch_event','/knu_batch/active_route','/debug/upper_stage_timing'):
        try: D[n].append((t,json.loads(m.data)))
        except Exception: pass
    elif n=='/motive/vehicle/odom_map': D[n].append((t,m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y))
    elif n=='/rosout': D[n].append((t,m.name,m.level,m.msg))
    else: D[n].append((t,list(m.data)))
pickle.dump(dict(D),open('decoded.pkl','wb'))
# grids: store with pose for accumulation
od=np.array([x for x in D['/motive/vehicle/odom_map']]);G=[]
for t,data in grids:
    j=min(bisect.bisect_left(od[:,0],t),len(od)-1)
    if abs(od[j,0]-t)<0.06: G.append((t,od[j,1],od[j,2],od[j,3],data))
pickle.dump(G,open('grids_sparse.pkl','wb'))
print({k:len(v) for k,v in D.items()},'grids kept',len(G))
