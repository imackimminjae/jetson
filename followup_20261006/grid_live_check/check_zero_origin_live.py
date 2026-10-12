"""Read-only live grid + planner/publisher parameter inspection; no control writes."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import time,json,os,re
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rcl_interfaces.srv import GetParameters,ListParameters

H=Path(__file__).resolve().parent/('zero_origin_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S'))
H.mkdir();print('OUTPUT',H,flush=True)
rows=[];events=[];last_meta=None
def grid(m):
 global last_meta
 a=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)
 r=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y
 x=ox+(np.arange(m.info.width)+.5)*r;y=oy+(np.arange(m.info.height)+.5)*r
 meta=dict(frame=m.header.frame_id,width=m.info.width,height=m.info.height,resolution=r,origin=[ox,oy],quaternion=[m.info.origin.orientation.x,m.info.origin.orientation.y,m.info.origin.orientation.z,m.info.origin.orientation.w],extent=[ox,ox+m.info.width*r,oy,oy+m.info.height*r])
 def stats(mask):
  v=a[mask];return dict(cells=int(v.size),free0=int(np.sum(v==0)),blocked100=int(np.sum(v==100)),unknown=int(np.sum(v<0)),other=int(np.sum((v!=0)&(v!=100)&(v>=0))))
 near=(x>=0)&(x<2.95);center=(np.abs(y)<=2)
 yr,xr=np.where(a==0)
 row=dict(received=time.time(),monotonic=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,meta=meta,values={str(int(v)):int(c) for v,c in zip(*np.unique(a,return_counts=True))},near=stats(np.broadcast_to(near,a.shape)),near_center=stats(center[:,None]&near[None,:]),centerline_values=[int(a[np.argmin(abs(y)),i]) for i in range(m.info.width)],free_extent=[ox+int(xr.min())*r,ox+(int(xr.max())+1)*r,oy+int(yr.min())*r,oy+(int(yr.max())+1)*r] if len(xr) else None)
 row['free_at_forward_distances']={str(d):stats(np.broadcast_to(np.arange(m.info.width)==int(np.argmin(abs(x-d))),a.shape)) for d in [.2,1,2,3,4.5,6,9,20,30,39]}
 rows.append(row);last_meta=meta
 if len(rows)==1:np.save(H/'first_grid.npy',a);(H/'first_metadata.json').write_text(json.dumps(meta,indent=2));print('FIRST',json.dumps(row),flush=True)
 np.save(H/'last_grid.npy',a)
def event(m):
 try:e=json.loads(m.data)
 except ValueError:return
 events.append({k:e.get(k) for k in ['t_start','x','y','yaw_rad','valid_plan','reason','horizon','k1_lane_constraint_excluded']})

rclpy.init();n=Node('codex_readonly_zero_origin_check',enable_rosout=False,start_parameter_services=False)
subs=[n.create_subscription(OccupancyGrid,'/bev/occupancy_grid',grid,qos_profile_sensor_data),n.create_subscription(String,'/debug/upper_branch_event',event,qos_profile_sensor_data)]
start=time.monotonic()
while time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.15)
nodes=n.get_node_names_and_namespaces();pubs=n.get_publishers_info_by_topic('/bev/occupancy_grid')
params={};attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'}
def read_params(full,names):
 c=n.create_client(GetParameters,full+'/get_parameters')
 if not c.wait_for_service(timeout_sec=.5):return {'status':'service_unavailable'}
 q=GetParameters.Request();q.names=names;f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=2)
 if not f.done() or not f.result():return {'status':'timeout'}
 return {key:getattr(v,attrs[v.type]) if v.type in attrs else None for key,v in zip(names,f.result().values)}
for name,ns in sorted(set(nodes)):
 if 'upper_planner' in name:
  full=ns.rstrip('/')+'/'+name
  params[full]=read_params(full,['skip_front_clipped_k1_lane_constraints','grid_origin_x_override_enabled','grid_origin_x_override_m','grid_positive_is_drivable','grid_value_threshold','upper_preview_steps','target_speed_mps'])
for full in sorted({p.node_namespace.rstrip('/')+'/'+p.node_name for p in pubs}):
 c=n.create_client(ListParameters,full+'/list_parameters');names=['x_min','x_max','y_min','y_max','near_fill_mode','near_free_distance','occupancy_invalid_as_unknown','occupancy_grid_rviz_standard_values']
 if c.wait_for_service(timeout_sec=.5):
  q=ListParameters.Request();q.depth=0;f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=2)
  if f.done() and f.result():names=[s for s in f.result().result.names if re.match(r'^(x_min|x_max|y_min|y_max|resolution|ground_z|near_|occupancy_|publish_bev_images|base_frame|camera_frame|road_|nonroad_|invalid_as_nonroad|keep_largest_component)',s)]
 params[full]=read_params(full,names)
span=rows[-1]['monotonic']-rows[0]['monotonic'] if len(rows)>1 else 0
result=dict(end_kst=datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),domain=os.environ.get('ROS_DOMAIN_ID'),frames=len(rows),rate_hz=(len(rows)-1)/span if span else None,nodes=nodes,publishers=[dict(name=p.node_name,namespace=p.node_namespace) for p in pubs],parameters=params,events=events,last_metadata=last_meta)
if rows:
 result['near_free_range']=[min(r['near']['free0'] for r in rows),max(r['near']['free0'] for r in rows)]
 result['near_center_free_range']=[min(r['near_center']['free0'] for r in rows),max(r['near_center']['free0'] for r in rows)]
 result['near_center_cells']=rows[-1]['near_center']['cells']
 result['near_unknown_range']=[min(r['near']['unknown'] for r in rows),max(r['near']['unknown'] for r in rows)]
(H/'frames.json').write_text(json.dumps(rows,indent=2));(H/'RESULT.json').write_text(json.dumps(result,indent=2));print('RESULT',json.dumps(result,indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
