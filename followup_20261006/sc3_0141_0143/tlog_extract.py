import sys, pickle, time, collections
from pymavlink import mavutil
src = sys.argv[1]; out = sys.argv[2]
# windows (local time KST) to keep
import datetime
def ts(s): return time.mktime(time.strptime(s, '%Y-%m-%d %H:%M:%S'))
windows = [(ts('2026-10-06 01:41:35'), ts('2026-10-06 01:43:45'))]
m = mavutil.mavlink_connection(src, dialect='common', robust_parsing=True)
data = collections.defaultdict(list)
want = {'COMMAND_LONG','COMMAND_ACK','HIL_ACTUATOR_CONTROLS','SERVO_OUTPUT_RAW','HIL_STATE_QUATERNION','LOCAL_POSITION_NED','ATTITUDE','MANUAL_CONTROL','VFR_HUD','ESTIMATOR_STATUS','HEARTBEAT','SYS_STATUS','EXTENDED_SYS_STATE','STATUSTEXT','GPS_RAW_INT','ODOMETRY'}
n=0
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    t = msg._timestamp
    typ = msg.get_type()
    if typ not in want: continue
    if not any(a <= t <= b for a,b in windows): continue
    d = msg.to_dict(); d['_t'] = t; d['_src'] = (msg.get_srcSystem(), msg.get_srcComponent())
    data[typ].append(d); n+=1
pickle.dump(dict(data), open(out,'wb'))
print('saved', n, {k:len(v) for k,v in data.items()})
