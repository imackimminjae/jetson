import sys, collections, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], dialect='common', robust_parsing=True)
counts = collections.Counter()
first = {}; last = {}
t0 = None; t1 = None
# window census: per-minute message counts for key types
win = collections.defaultdict(collections.Counter)
keys = ('COMMAND_LONG','COMMAND_ACK','HIL_STATE_QUATERNION','LOCAL_POSITION_NED','ATTITUDE','HIL_ACTUATOR_CONTROLS','SERVO_OUTPUT_RAW','ACTUATOR_OUTPUT_STATUS','VISION_POSITION_ESTIMATE','HEARTBEAT','STATUSTEXT','ESTIMATOR_STATUS','VFR_HUD','GPS_RAW_INT','HIL_SENSOR','HIL_GPS','SYS_STATUS')
while True:
    msg = m.recv_match(blocking=False)
    if msg is None: break
    t = getattr(msg, '_timestamp', None)
    typ = msg.get_type()
    counts[typ] += 1
    if t0 is None: t0 = t
    t1 = t
    if typ not in first: first[typ] = t
    last[typ] = t
    if typ in keys:
        win[int(t//60)*60][typ] += 1
print("range", time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t0)), '->', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t1)))
for k,v in counts.most_common(60):
    print(f"{k:32s} {v:8d}  {time.strftime('%H:%M:%S', time.localtime(first[k]))} - {time.strftime('%H:%M:%S', time.localtime(last[k]))}")
print("== per-minute windows (key types) ==")
for w in sorted(win):
    print(time.strftime('%m-%d %H:%M', time.localtime(w)), dict(win[w]))
