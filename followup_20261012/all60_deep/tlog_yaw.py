import math,numpy as np,pickle
from pymavlink import mavutil
m=mavutil.mavlink_connection('/home/imac/ros2_ws/mav.tlog',robust_parsing=True)
att=[];tru=[];last_a=0;last_t=0
while True:
    msg=m.recv_match(type=['ATTITUDE','HIL_STATE_QUATERNION'],blocking=False)
    if msg is None: break
    t=getattr(msg,'_timestamp',0)
    if msg.get_type()=='ATTITUDE':
        if t-last_a>=0.1: att.append((t,msg.yaw));last_a=t
    else:
        if t-last_t>=0.1:
            w,x,y,z=msg.attitude_quaternion;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));tru.append((t,yaw));last_t=t
att=np.array(att);tru=np.array(tru);pickle.dump(dict(att=att,tru=tru),open('tlog_yaw.pkl','wb'))
print('ATTITUDE',len(att),'TRUTH',len(tru),'span',att[0,0] if len(att) else None,att[-1,0] if len(att) else None)
