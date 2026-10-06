import os,sqlite3,subprocess,time,signal,json
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String,Float64MultiArray
root=Path('/home/imac/ros2_ws');analysis=root/'drive_debug_20260923_232054_light/analysis'
db=sqlite3.connect(f'file:{analysis.parent}/bag/bag_0.db3?mode=ro',uri=True)
wanted=['/navigation/global_path','/motive/vehicle/odom_map','/bev/occupancy_grid']
types={i:(name,get_message(kind)) for i,name,kind in db.execute('select id,name,type from topics') if name in wanted}
messages=[(t/1e9,ident,deserialize_message(blob,types[ident][1])) for ident,t,blob in db.execute('select topic_id,timestamp,data from messages order by timestamp') if ident in types];db.close()
rclpy.init();node=rclpy.create_node('isolated_sway_replay');pubs={}
for ident,(name,cls) in types.items():pubs[ident]=node.create_publisher(cls,name,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL if name==wanted[0] else DurabilityPolicy.VOLATILE))
events=[];lower=[];stages=[]
subs=[node.create_subscription(String,'/debug/upper_branch_event',lambda m: events.append(json.loads(m.data)),10),node.create_subscription(Float64MultiArray,'/debug/lower_mpc_trace',lambda m: lower.append(list(m.data)),10),node.create_subscription(String,'/debug/upper_stage_timing',lambda m: stages.append(json.loads(m.data)),QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))]
procs=[];logs=[]
try:
 for exe in ['upper_planner_node','lower_tracking_mpc_node']:
  log=open('/tmp/requested_replay_'+exe+'.log','w');logs.append(log)
  cmd=[str(root/'install/virtual_control/lib/virtual_control'/exe),'--ros-args','--params-file',str(root/'src/virtual_control/config/tracking_control_split.yaml')]
  if exe=='lower_tracking_mpc_node':cmd+=['-p','mavlink_enable:=false','-p','pixhawk_output_backend:=disabled','-p','publish_pwm:=false']
  procs.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT))
 # Discovery without publishing any vehicle command to domain 1.
 deadline=time.monotonic()+1.5
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
 start=time.monotonic();bag_start=messages[0][0];offset=time.time()-bag_start
 for timestamp,ident,msg in messages:
  due=start+timestamp-bag_start
  while time.monotonic()<due:rclpy.spin_once(node,timeout_sec=min(.01,max(0,due-time.monotonic())))
  msg.header.stamp=node.get_clock().now().to_msg();pubs[ident].publish(msg);rclpy.spin_once(node,timeout_sec=0)
 deadline=time.monotonic()+.15
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
finally:
 for p in procs:p.send_signal(signal.SIGINT)
 for p in procs:p.wait(timeout=10)
 for log in logs:log.close()
 node.destroy_node();rclpy.shutdown()
(analysis/'requested_changes_replay.json').write_text(json.dumps(dict(events=events,lower=lower,stages=stages,wall_to_bag_offset=offset),indent=2))
from collections import Counter
print('upper',len(events),Counter(e['reason'] for e in events),'horizon',Counter(e['horizon'] for e in events));print('lower',len(lower),Counter(x[12] for x in lower))
