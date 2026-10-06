from pymavlink import mavutil
import numpy as np
m=mavutil.mavlink_connection('/tmp/sih_repair_mav.tlog');truth=[];att=[];cmd=[]
while True:
 x=m.recv_match(type=['HIL_STATE_QUATERNION','ATTITUDE','COMMAND_LONG'])
 if x is None:break
 t=x._timestamp
 if t<1790173245:continue
 if t>1790173324:break
 kind=x.get_type()
 if kind=='HIL_STATE_QUATERNION':truth.append([t,x.yawspeed,(x.vx*x.vx+x.vy*x.vy)**.5*.01,*x.attitude_quaternion])
 elif kind=='ATTITUDE':att.append([t,x.yaw,x.yawspeed])
 elif kind=='COMMAND_LONG' and x.command==187:cmd.append([t,x.param2])
np.savez('/home/imac/ros2_ws/drive_debug_20260923_232054_light/analysis/sway_truth.npz',truth=np.array(truth),att=np.array(att),cmd=np.array(cmd))
print('truth/att/cmd',len(truth),len(att),len(cmd))
