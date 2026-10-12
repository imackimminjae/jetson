import os
os.environ['MAVLINK20']='1'
from pathlib import Path
from pymavlink import mavutil
from collections import Counter
from datetime import datetime
import json,math
p=Path(__file__).resolve().parent;c=mavutil.mavlink_connection(str(p/'telemetry_snapshot.tlog'),dialect='common',robust_parsing=True);counts=Counter();latest={};warnings=[];params={};first=None;last=None
while True:
 m=c.recv_match()
 if m is None:break
 ty=m.get_type()
 if ty=='BAD_DATA':continue
 ts=getattr(m,'_timestamp',None)
 if ts is not None:first=ts if first is None else first;last=ts
 counts[ty]+=1
 if ty in ['HEARTBEAT','ATTITUDE','LOCAL_POSITION_NED','GPS_RAW_INT','HIGHRES_IMU','RAW_IMU','SCALED_IMU','SCALED_IMU2','SCALED_IMU3','HIL_STATE_QUATERNION','ESTIMATOR_STATUS','EKF_STATUS_REPORT','AUTOPILOT_VERSION','SYS_STATUS','VIBRATION']:
  latest[ty]={'time':datetime.fromtimestamp(ts).isoformat() if ts else None,'data':m.to_dict()}
 if ty=='PARAM_VALUE':params[m.param_id]=m.to_dict()
 if ty=='STATUSTEXT':warnings.append({'time':datetime.fromtimestamp(ts).isoformat(),'severity':m.severity,'text':m.text})
out={'first':datetime.fromtimestamp(first).isoformat(),'last':datetime.fromtimestamp(last).isoformat(),'counts':dict(counts),'latest':latest,'recent_text':warnings[-35:]}
(p/'telemetry_summary.json').write_text(json.dumps(out,indent=2,default=str));(p/'parameter_messages.json').write_text(json.dumps(params,indent=2,default=str));print('range',out['first'],out['last']);print('types',counts)
for ty in ['ATTITUDE','HIL_STATE_QUATERNION','GPS_RAW_INT','HIGHRES_IMU','ESTIMATOR_STATUS','AUTOPILOT_VERSION']:
 print(ty,latest.get(ty))
print('recent_text',warnings[-15:])
