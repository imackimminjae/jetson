import rclpy,time,json
from pathlib import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('verify_sih_mag_fix');states=[];counts={'pose':0,'ekf':0,'map':0};stamps={k:[] for k in counts}
def cb(k):
 def f(m):counts[k]+=1;stamps[k].append(time.monotonic())
 return f
n.create_subscription(String,'/px4_ekf_bridge/status',lambda m:states.append(json.loads(m.data)),qos_profile_sensor_data)
for typ,name,k in [(PoseStamped,'/px4/sih/odom','pose'),(Odometry,'/px4/ekf_odom','ekf'),(Odometry,'/motive/vehicle/odom_map','map')]:n.create_subscription(typ,name,cb(k),qos_profile_sensor_data)
end=time.monotonic()+15
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
rates={k:(len(t)-1)/(t[-1]-t[0]) if len(t)>1 else 0 for k,t in stamps.items()};out={'counts':counts,'rates_hz':rates,'states':states};Path(__file__).resolve().parent.joinpath('ros_after_fix.json').write_text(json.dumps(out,indent=2));print('counts',counts,'rates',rates);print('states',[(s['connected'],s['sih_heading_guard_state'],s['ekf_minus_sih_yaw_deg']) for s in states]);n.destroy_node();rclpy.shutdown()
