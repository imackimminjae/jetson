import os,time,json
os.environ['MAVLINK20']='1'
from pathlib import Path
from pymavlink import mavutil
p=Path(__file__).resolve().parent
m=mavutil.mavlink_connection('udpin:127.0.0.1:14540',source_system=246,source_component=191,dialect='common')
logs=[];last={};text=[];shell=[]
def poll(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  z=m.recv_match(blocking=True,timeout=.15)
  if not z:continue
  t=z.get_type()
  if t=='LOG_ENTRY':logs.append(z.to_dict())
  if t=='SERIAL_CONTROL':shell.append(bytes(z.data[:z.count]).decode(errors='replace'))
  if t=='STATUSTEXT':text.append(z.to_dict())
  if t in ['HEARTBEAT','ATTITUDE','HIL_STATE_QUATERNION','LOCAL_POSITION_NED']:last[t]=z.to_dict()
poll(2)
if 'HEARTBEAT' not in last:raise RuntimeError('No heartbeat; no queries sent')
print('heartbeat',last['HEARTBEAT'],flush=True)
poll(.1)
print('log entries',logs,flush=True)
commands=['sih status','sensor_mag_sim status','listener sensor_mag 1','listener vehicle_attitude_groundtruth 1','listener vehicle_global_position_groundtruth 1','listener estimator_status_flags 1','listener estimator_sensor_bias 1','ls /fs/microsd/log']
try:
 for cmd in commands:
  start=len(shell);buf=(cmd+'\n').encode();flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND|mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
  m.mav.serial_control_send(10,flags,0,0,len(buf),list(buf)+[0]*(70-len(buf)));poll(1.6)
  # Request any remaining shell output without sending another command.
  m.mav.serial_control_send(10,flags,0,0,0,[0]*70);poll(.5)
  print('COMMAND',cmd,'\n'+''.join(shell[start:]),flush=True)
finally:
 m.mav.serial_control_send(10,0,0,0,0,[0]*70)
 m.mav.log_request_end_send(1,1)
 m.close()
(p/'board_sensor_query.json').write_text(json.dumps({'log_entries':logs,'last':last,'statustext':text,'shell':''.join(shell)},indent=2));(p/'board_sensor_shell.txt').write_text(''.join(shell))
