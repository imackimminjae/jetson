"""Read-only 15-second grid subscriber: no control publishers or service calls."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from collections import Counter
import os,time,json,hashlib
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
H=Path(__file__).resolve().parent;stamp=datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S');out=H/stamp;out.mkdir()
rows=[];patterns={};first_mono=None
config={'topic':'/bev/occupancy_grid','ROS_DOMAIN_ID':os.environ.get('ROS_DOMAIN_ID'),'ROS_LOCALHOST_ONLY':os.environ.get('ROS_LOCALHOST_ONLY'),'start_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),'read_only':True}
(out/'session.json').write_text(json.dumps(config,indent=2))
def callback(m):
 global first_mono
 now=time.monotonic()
 if first_mono is None:first_mono=now
 a=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width);known=a>=0;ys,xs=np.where(known);o=m.info.origin.position;q=m.info.origin.orientation;r=m.info.resolution
 bbox=[int(xs.min()),int(xs.max()),int(ys.min()),int(ys.max())] if len(xs) else None
 ext=[o.x+bbox[0]*r,o.x+(bbox[1]+1)*r,o.y+bbox[2]*r,o.y+(bbox[3]+1)*r] if bbox else None
 holes=(bbox[1]-bbox[0]+1)*(bbox[3]-bbox[2]+1)-int(known.sum()) if bbox else 0
 meta={'size':[m.info.width,m.info.height],'resolution_m':r,'frame':m.header.frame_id,'origin':[o.x,o.y,o.z],'origin_quaternion':[q.x,q.y,q.z,q.w],'known_bbox_indices':bbox,'known_extent_grid_axes_m':ext,'known_cells':int(known.sum()),'unknown_cells':int((~known).sum()),'internal_unknown_holes':holes,'known_mask_sha256':hashlib.sha256(np.packbits(known).tobytes()).hexdigest()}
 key=hashlib.sha256(json.dumps(meta,sort_keys=True).encode()).hexdigest()[:16];patterns[key]=meta
 rows.append({'received_wall_ns':time.time_ns(),'received_monotonic':now,'header_stamp_ns':m.header.stamp.sec*1000000000+m.header.stamp.nanosec,'pattern_id':key,'free_cells':int((a==0).sum()),'grid_sha256':hashlib.sha256(a.tobytes()).hexdigest()})
 if len(rows)==1:
  np.save(out/'first_grid.npy',a);(out/'first_metadata.json').write_text(json.dumps(meta,indent=2));print('FIRST_FRAME',json.dumps(meta),flush=True)
 np.save(out/'last_grid.npy',a)
try:
 rclpy.init();node=Node('codex_readonly_grid_extent_check',enable_rosout=False,start_parameter_services=False)
 sub=node.create_subscription(OccupancyGrid,config['topic'],callback,qos_profile_sensor_data)
 start=time.monotonic()
 while time.monotonic()-start<30 and (first_mono is None or time.monotonic()-first_mono<15):rclpy.spin_once(node,timeout_sec=.2)
 publishers=[{'node_name':p.node_name,'node_namespace':p.node_namespace,'topic_type':p.topic_type} for p in node.get_publishers_info_by_topic(config['topic'])]
 span=rows[-1]['received_monotonic']-rows[0]['received_monotonic'] if len(rows)>1 else 0
 result={'status':'received' if rows else 'no_frames','frames':len(rows),'received_span_sec':span,'received_rate_hz':(len(rows)-1)/span if span else None,'pattern_counts':dict(Counter(r['pattern_id'] for r in rows)),'patterns':patterns,'publishers':publishers,'zero_header_stamp_frames':sum(r['header_stamp_ns']==0 for r in rows),'unique_grid_payloads':len(set(r['grid_sha256'] for r in rows)),'end_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat()}
 node.destroy_node();rclpy.shutdown()
except Exception as e:result={'status':'error','type':type(e).__name__,'message':str(e),'frames':len(rows)}
(out/'frames.json').write_text(json.dumps(rows,indent=2));(out/'RESULT.json').write_text(json.dumps(result,indent=2));print('RESULT_PATH',str(out/'RESULT.json'));print(json.dumps(result,indent=2),flush=True)
