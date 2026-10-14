"""Passive, bounded ROS subscriber: no commands, services, or parameter changes."""
import argparse, json, time
from pathlib import Path
from datetime import datetime, timezone
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image, CameraInfo

ap=argparse.ArgumentParser()
ap.add_argument('--seconds',type=float,default=60)
ap.add_argument('--discovery-seconds',type=float,default=15)
ap.add_argument('--output',required=True)
ap.add_argument('--with-camera',action='store_true')
args=ap.parse_args()
rclpy.init()
node=rclpy.create_node('codex_bev_receive_probe',enable_rosout=False,start_parameter_services=False)
started=time.monotonic();epoch=time.time();first=None;records={};subscriptions=[]

def receive(key):
    def cb(message):
        global first
        now=time.monotonic()
        if key.startswith('bev') and first is None:first=now
        records.setdefault(key,[]).append(now)
    return cb

for key,typ,topic,policy in [
    ('bev_best_effort',OccupancyGrid,'/bev/occupancy_grid',ReliabilityPolicy.BEST_EFFORT),
    ('bev_reliable',OccupancyGrid,'/bev/occupancy_grid',ReliabilityPolicy.RELIABLE),
    ('odom',Odometry,'/motive/vehicle/odom_map',ReliabilityPolicy.BEST_EFFORT),
    ('upper_timing',String,'/debug/upper_stage_timing',ReliabilityPolicy.BEST_EFFORT),
    ('bev_publisher_rosout',Log,'/rosout',ReliabilityPolicy.BEST_EFFORT)]:
    qos=QoSProfile(depth=10,reliability=policy,durability=DurabilityPolicy.VOLATILE)
    if key=='bev_publisher_rosout':
        def cb(m):
            if m.name=='rgb_to_occupancy':records.setdefault('bev_publisher_rosout',[]).append(time.monotonic())
        subscriptions.append(node.create_subscription(typ,topic,cb,qos))
    else:subscriptions.append(node.create_subscription(typ,topic,receive(key),qos,raw=True))
if args.with_camera:
    for key,typ,topic in [('rgb',Image,'/rgb'),('camera_info',CameraInfo,'/camera_info'),('isaac_odom',Odometry,'/odom')]:
        subscriptions.append(node.create_subscription(typ,topic,receive(key),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE),raw=True))
last=started
try:
    while True:
        now=time.monotonic()
        if first is None and now-started>=args.discovery_seconds:break
        if first is not None and now-first>=args.seconds:break
        rclpy.spin_once(node,timeout_sec=.05)
        if now-last>=10:
            print(json.dumps({'elapsed_s':round(now-started,1),'counts':{k:len(v) for k,v in records.items()}},ensure_ascii=False),flush=True);last=now
    finished=time.monotonic();summaries={}
    for key,ts in records.items():
        arr=np.asarray(ts);dt=np.diff(arr)
        summaries[key]={'count':len(ts),'first_elapsed_s':ts[0]-started,'last_elapsed_s':ts[-1]-started,'trailing_silence_s':finished-ts[-1],'mean_rate_hz':float((len(ts)-1)/(ts[-1]-ts[0])) if len(ts)>1 else None,'max_gap_s':float(dt.max()) if len(dt) else None,'p95_gap_s':float(np.percentile(dt,95)) if len(dt) else None,'gaps_over_0_5s':int(sum(dt>.5)),'gaps_over_1s':int(sum(dt>1)),'long_gaps':[{'elapsed_s':float(ts[i]-started),'gap_s':float(dt[i])} for i in np.flatnonzero(dt>.5)]}
    endpoints=[]
    for info in node.get_publishers_info_by_topic('/bev/occupancy_grid'):
        endpoints.append({'node':info.node_name,'namespace':info.node_namespace,'qos':str(info.qos_profile)})
    report={'start_utc':datetime.fromtimestamp(epoch,timezone.utc).isoformat(),'duration_s':finished-started,'observed_s_from_first_bev':finished-first if first else 0,'domain_id':__import__('os').environ.get('ROS_DOMAIN_ID'),'rmw':rclpy.get_rmw_implementation_identifier(),'topics':summaries,'bev_publishers':endpoints,'node_names':node.get_node_names_and_namespaces(),'receipt_elapsed_s':{k:[t-started for t in ts] for k,ts in records.items()}}
    Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='receipt_elapsed_s'},ensure_ascii=False,indent=2),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
