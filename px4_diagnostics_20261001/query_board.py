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
m.mav.log_request_list_send(1,1,0,65535);poll(3)
print('log entries',logs,flush=True)
commands=['ver all','sensors status','ekf2 status','listener vehicle_magnetometer 1','listener sensor_selection 1','listener estimator_selector_status 1','logger status']
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
(p/'board_query.json').write_text(json.dumps({'log_entries':logs,'last':last,'statustext':text,'shell':''.join(shell)},indent=2));(p/'board_shell.txt').write_text(''.join(shell))
