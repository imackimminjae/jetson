"""Finite read-only board queries. No heartbeat, arm, actuator or parameter commands."""
import os,time,json,subprocess,argparse,ast
from pathlib import Path
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
root=Path(__file__).resolve().parent
module=ast.parse((root/'read_board.py').read_text())
GROUPS=next(ast.literal_eval(n.value) for n in module.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='GROUPS' for t in n.targets))
p=argparse.ArgumentParser();p.add_argument('--group',choices=GROUPS,required=True);a=p.parse_args();device='/dev/ttyACM0'
r=subprocess.run(['fuser',device],capture_output=True,text=True)
if r.returncode!=1:raise SystemExit('Serial port is in use or ownership cannot be verified: '+r.stdout+r.stderr)
m=mavutil.mavlink_connection(device,baud=115200,source_system=246,source_component=191,dialect='common');m.port.exclusive=True
records=[];shell=[]
def poll(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:
  x=m.recv_match(blocking=True,timeout=.1)
  if x is None:continue
  record={**x.to_dict(),'host_time':time.time(),'source_system':x.get_srcSystem(),'source_component':x.get_srcComponent()};records.append(record)
  if x.get_type()=='SERIAL_CONTROL':shell.append(bytes(x.data[:x.count]).decode(errors='replace'))
flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND|mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
try:
 poll(2)
 if not any(r['mavpackettype']=='HEARTBEAT' and r['source_system']==1 for r in records):raise RuntimeError('No board heartbeat; no shell commands sent')
 if a.group in ('log_index','record_check'):
  m.mav.log_request_list_send(1,1,0,65535);poll(10)
  entries=[r for r in records if r['mavpackettype']=='LOG_ENTRY'];print('LOG_ENTRIES',json.dumps(entries),flush=True)
 for cmd in GROUPS[a.group]:
  first=len(shell);buf=(cmd+'\n').encode();assert len(buf)<=70
  m.mav.serial_control_send(10,flags,0,0,len(buf),list(buf)+[0]*(70-len(buf)));poll(1.6)
  m.mav.serial_control_send(10,flags,0,0,0,[0]*70);poll(.6)
  print('COMMAND',cmd,'\n'+''.join(shell[first:]),flush=True)
finally:
 if a.group in ('log_index','record_check'):m.mav.log_request_end_send(1,1)
 m.mav.serial_control_send(10,0,0,0,0,[0]*70);m.close()
 (root/f'usb_{a.group}_shell.txt').write_text(''.join(shell))
 (root/f'usb_{a.group}_responses.json').write_text(json.dumps({'commands':GROUPS[a.group],'records':records},indent=2,default=lambda x:list(x) if isinstance(x,(bytes,bytearray)) else str(x)))
