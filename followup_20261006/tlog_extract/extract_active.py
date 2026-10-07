"""Extract key MAVLink types for every active drive window (COMMAND_LONG 187 traffic) after a start time."""
import sys, pickle, time, collections
from pymavlink import mavutil
src, out, start = sys.argv[1], sys.argv[2], time.mktime(time.strptime(sys.argv[3], '%Y-%m-%d %H:%M:%S'))
want = {'COMMAND_LONG','COMMAND_ACK','HIL_ACTUATOR_CONTROLS','SERVO_OUTPUT_RAW','HIL_STATE_QUATERNION','LOCAL_POSITION_NED','ATTITUDE','MANUAL_CONTROL','VFR_HUD','ESTIMATOR_STATUS','HEARTBEAT','SYS_STATUS','EXTENDED_SYS_STATE','STATUSTEXT','GPS_RAW_INT','ODOMETRY'}
m = mavutil.mavlink_connection(src, dialect='common', robust_parsing=True)
buf = []; cmd_t = []
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    t = msg._timestamp
    if t < start or msg.get_type() not in want: continue
    d = msg.to_dict(); d['_t'] = t; d['_src'] = (msg.get_srcSystem(), msg.get_srcComponent()); buf.append(d)
    if d['mavpackettype'] == 'COMMAND_LONG' and d.get('command') == 187: cmd_t.append(t)
# active windows: command traffic clusters, gap > 20 s splits, +/-15 s margin
wins = []
for t in sorted(cmd_t):
    if wins and t - wins[-1][1] <= 20: wins[-1][1] = t
    else: wins.append([t, t])
wins = [(a - 15, b + 15) for a, b in wins]
data = collections.defaultdict(list)
for d in buf:
    if any(a <= d['_t'] <= b for a, b in wins): data[d['mavpackettype']].append(d)
pickle.dump({'windows': wins, 'data': dict(data)}, open(out, 'wb'))
fmt = lambda t: time.strftime('%H:%M:%S', time.localtime(t))
print('windows:', [(fmt(a), fmt(b)) for a, b in wins]); print({k: len(v) for k, v in data.items()})
