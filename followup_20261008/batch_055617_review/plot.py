from pathlib import Path
import json,os,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
H=Path(__file__).resolve().parent;s=json.loads((H/'summary.json').read_text());details=json.loads((H/'details.json').read_text());phases=s['phases'];route=np.array(s['route']);es=[(e['t'],e) for e in json.loads((H/'events.json').read_text())]
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=s['session']+'/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/motive/vehicle/odom_map','/debug/lower_mpc_trace']))
od=[];ot=[];lo=[];lt=[]
while r.has_next():
 n,blob,t=r.read_next()
 if n.endswith('odom_map'):
  m=deserialize_message(blob,Odometry);ot.append(t/1e9);od.append([m.pose.pose.position.x,m.pose.pose.position.y])
 else:
  m=deserialize_message(blob,Float64MultiArray);lt.append(t/1e9);lo.append(m.data)
ot=np.array(ot);od=np.array(od);lt=np.array(lt);lo=np.array(lo)
script=(H/'analyze.py').read_text().split('# Figures:')[1]
script=script[script.index("os.environ['MPLCONFIGDIR']"):]
script=script.replace("b=next(s['t'] for s in s1 if s['phase']=='coasting')", "b=details['first_neutral_stamp']")
script=script.replace('목표 영역 접근 판정','첫 중립 출력').replace('주행 종료 판정','첫 중립 출력')
exec(script)
