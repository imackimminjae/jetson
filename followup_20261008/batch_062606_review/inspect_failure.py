from pathlib import Path
import json,math,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
H=Path(__file__).resolve().parent;s=json.loads((H/'summary.json').read_text());es=json.loads((H/'events.json').read_text());wanted=[]
for i,xx in [(1,239.5),(4,238.5),(5,231.9),(5,233.1),(5,234.5),(5,236.3),(5,238.5),(5,240.9),(6,239.3)]:
 z=s['attempts'][i-1];e=min([e for e in es if z['start']<=e['t_start']<z['end']],key=lambda e:abs(e['x']-xx));wanted.append((i,e))
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=s['session']+'/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/debug/upper_interval_info','/debug/upper_interval_centering_state']));infos=[];bs=[]
while r.has_next():
 n,b,t=r.read_next();m=deserialize_message(b,Float64MultiArray)
 (infos if n.endswith('info') else bs).append((t/1e9,np.array(m.data)))
out=[]
for i,e in wanted:
 t=e['t_emit'];it,v=min(infos,key=lambda z:abs(z[0]-t));rows=v.reshape(-1,12);bt,B=min(bs,key=lambda z:abs(z[0]-t));p=np.array([e['x'],e['y']]);y=e['yaw_rad'];R=np.array([[math.cos(y),-math.sin(y)],[math.sin(y),math.cos(y)]]);ref=(np.array(e['preview_world'])-p)@R;row=dict(attempt=i,t=e['t_start'],x=e['x'],y=e['y'],yaw_deg=math.degrees(y),ref_body=ref.tolist(),k5_6_info=rows[rows[:,0]>=5].tolist(),B=B.tolist(),guard=e['side_boundary_tail_start'],valid=e['valid_plan'],end=e['published_world'][-1] if e['published_world'] else None,grid_side_clearance_m=float(20-ref[-1,1]),guard_threshold_m=float(np.linalg.norm(ref[-1]-ref[-2])+.3125),info_age_sec=t-it)
 out.append(row)
 print(i,round(e['x'],2),'body ref6',ref[-1],'edge margin',row['grid_side_clearance_m'],'limit',row['guard_threshold_m'],'B',B[1:3]);print('rows',rows[rows[:,0]>=5])
(H/'failure_comparison.json').write_text(json.dumps(out,indent=2)+'\n')
