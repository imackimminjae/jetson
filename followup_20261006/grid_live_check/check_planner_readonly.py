"""Bounded read-only subscriptions and get_parameters; never sends control."""
import rclpy,time,json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid
import numpy as np
out=Path(__file__).resolve().parent / ('status_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S'));out.mkdir()
rclpy.init();n=Node('codex_readonly_planner_status',enable_rosout=False,start_parameter_services=False)
events=[];logs=[];grids=[]
def event(m):
 try: value=json.loads(m.data)
 except Exception:value=m.data
 events.append({'received_wall':time.time(),'data':value})
def log(m):
 if 'upper' in m.name or 'planner' in m.name:logs.append({'received_wall':time.time(),'name':m.name,'level':m.level,'msg':m.msg})
def grid(m):
 a=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width);y,x=np.where(a==0);r=m.info.resolution;o=m.info.origin.position
 grids.append({'received_wall':time.time(),'free':int(len(x)),'unknown':int((a<0).sum()),'free_bbox':[int(x.min()),int(x.max()),int(y.min()),int(y.max())] if len(x) else None,'free_extent':[o.x+int(x.min())*r,o.x+(int(x.max())+1)*r,o.y+int(y.min())*r,o.y+(int(y.max())+1)*r] if len(x) else None,'near_first_9m_free':int((a[:,:int(9/r)]==0).sum())})
 if len(grids)==1:np.save(out/'first_grid.npy',a)
 np.save(out/'last_grid.npy',a)
subs=[n.create_subscription(String,'/debug/upper_branch_event',event,qos_profile_sensor_data),n.create_subscription(Log,'/rosout',log,qos_profile_sensor_data),n.create_subscription(OccupancyGrid,'/bev/occupancy_grid',grid,qos_profile_sensor_data)]
start=time.monotonic()
while time.monotonic()-start<12:rclpy.spin_once(n,timeout_sec=.15)
nodes=n.get_node_names_and_namespaces();params={}
for name,ns in nodes:
 if 'upper' not in name:continue
 service=ns.rstrip('/')+'/'+name+'/get_parameters';c=n.create_client(GetParameters,service)
 if not c.wait_for_service(timeout_sec=.3):continue
 q=GetParameters.Request();q.names=['grid_origin_x_override_enabled','grid_origin_x_override_m','grid_positive_is_drivable','grid_value_threshold','upper_preview_steps','target_speed_mps','input_timeout_sec'];f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=1)
 if f.done() and f.result():
  attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'};params[name]={k:getattr(v,attrs[v.type]) if v.type in attrs else None for k,v in zip(q.names,f.result().values)}
res={'nodes':nodes,'upper_parameters':params,'event_count':len(events),'logs':logs,'events':events,'grids':grids}
(out/'capture.json').write_text(json.dumps(res,indent=2));print('OUTPUT',out);print('NODES',nodes);print('PARAMS',params);print('EVENTS',len(events));print('EVENT_SAMPLE',json.dumps(events[-2:]));print('LOG_SAMPLE',json.dumps(logs[-12:]));print('GRIDS',len(grids),'first',grids[:1],'last',grids[-1:]);n.destroy_node();rclpy.shutdown()
