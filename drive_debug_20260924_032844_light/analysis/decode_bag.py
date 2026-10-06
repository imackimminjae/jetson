import sqlite3,pickle,json
from pathlib import Path
from collections import defaultdict,Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path('/home/imac/ros2_ws/drive_debug_20260924_032844_light')
(root/'analysis').mkdir(exist_ok=True)
c=sqlite3.connect(f'file:{root}/bag/bag_0.db3?mode=ro',uri=True)
topics={i:(n,t) for i,n,t in c.execute('select id,name,type from topics')}
classes={};data=defaultdict(list)
def stamp(s):return s.sec+s.nanosec*1e-9
def xy(p):return [p.pose.position.x,p.pose.position.y]
def quat(q):return [q.x,q.y,q.z,q.w]
for i,t,blob in c.execute('select topic_id,timestamp,data from messages order by timestamp'):
 n,kind=topics[i]
 if kind in ('sensor_msgs/msg/Image','sensor_msgs/msg/CameraInfo','tf2_msgs/msg/TFMessage') or n=='/parameter_events':continue
 if kind not in classes:classes[kind]=get_message(kind)
 m=deserialize_message(blob,classes[kind]);o={'t':t/1e9}
 if kind=='nav_msgs/msg/OccupancyGrid':o.update(stamp=stamp(m.header.stamp),frame=m.header.frame_id,res=m.info.resolution,width=m.info.width,height=m.info.height,origin=[m.info.origin.position.x,m.info.origin.position.y],q=quat(m.info.origin.orientation),grid=np.array(m.data).reshape(m.info.height,m.info.width))
 elif kind=='nav_msgs/msg/Path':o.update(stamp=stamp(m.header.stamp),frame=m.header.frame_id,xy=np.array([xy(p) for p in m.poses]))
 elif kind=='imac_interfaces/msg/PathWithArcLength':o.update(stamp=stamp(m.path.header.stamp),frame=m.path.header.frame_id,xy=np.array([xy(p) for p in m.path.poses]),s=np.array(m.s))
 elif kind=='nav_msgs/msg/Odometry':o.update(stamp=stamp(m.header.stamp),frame=m.header.frame_id,xy=[m.pose.pose.position.x,m.pose.pose.position.y],q=quat(m.pose.pose.orientation),v=[m.twist.twist.linear.x,m.twist.twist.linear.y],omega=m.twist.twist.angular.z)
 elif kind=='geometry_msgs/msg/PoseStamped':o.update(stamp=stamp(m.header.stamp),frame=m.header.frame_id,xy=[m.pose.position.x,m.pose.position.y],q=quat(m.pose.orientation))
 elif kind=='std_msgs/msg/Float64MultiArray':o.update(data=np.array(m.data))
 elif kind=='std_msgs/msg/String':
  try:o.update(data=json.loads(m.data))
  except ValueError:o.update(data=m.data)
 elif kind=='rcl_interfaces/msg/Log':o.update(stamp=stamp(m.stamp),name=m.name,level=m.level,msg=m.msg)
 elif kind in ('std_msgs/msg/Bool','std_msgs/msg/Float64'):o.update(data=m.data)
 else:continue
 data[n].append(o)
pickle.dump(dict(data),open(root/'analysis/decoded.pkl','wb'))
def clock(t):return datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
for n in ['/bev/occupancy_grid','/motive/vehicle/odom_map','/planner/lower_reference_path/with_arclength','/debug/upper_branch_event','/debug/lower_mpc_trace']:
 vals=data[n];ts=np.array([x['t'] for x in vals]);d=np.diff(ts)
 print(n,'count',len(vals),'start/end',clock(ts[0]),clock(ts[-1]),'gap median/max',np.median(d),max(d))
 if n=='/bev/occupancy_grid':print('grid',[(x['res'],x['width'],x['height'],x['origin'],x['q']) for x in vals[:1]],'gaps>1s',[(clock(ts[i]),round(g,3)) for i,g in enumerate(d) if g>1])
lower=np.array([x['data'] for x in data['/debug/lower_mpc_trace']]);print('LOWER modes',Counter(lower[:,12]),'speed min/med/max',np.quantile(lower[:,4],[0,.5,1]),'valid steering saturation',np.mean(abs(lower[lower[:,10]==1,9])>.95))
for mode in sorted(set(lower[:,12])):
 x=lower[lower[:,12]==mode];print('mode',mode,'count',len(x),'first,last',clock(x[0,0]),clock(x[-1,0]))
for e in data['/debug/upper_branch_event']:
 x=e['data'];print('UPPER',clock(e['t']),x['reason'],x['horizon'],'xy',round(x['x'],2),round(x['y'],2),'yaw',round(np.degrees(x['yaw_rad']),1),'status',x['status'])
print('WARNINGS')
for x in data['/rosout']:
 if x['level']>=30:print(clock(x['t']),x['name'],x['msg'])
