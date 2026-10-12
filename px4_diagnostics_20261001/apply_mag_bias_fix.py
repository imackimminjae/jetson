import os,time,json,re,math
os.environ['MAVLINK20']='1'
from pathlib import Path
from pymavlink import mavutil
p=Path(__file__).resolve().parent;m=mavutil.mavlink_connection('udpin:127.0.0.1:14540',source_system=246,source_component=191,dialect='common');last={};shell=[];events=[]
def poll(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  z=m.recv_match(blocking=True,timeout=.15)
  if z is None:continue
  if z.get_srcSystem()!=1 or z.get_srcComponent()!=1:continue
  ty=z.get_type()
  if ty=='SERIAL_CONTROL':shell.append(bytes(z.data[:z.count]).decode(errors='replace'))
  elif ty in ['HEARTBEAT','HIL_STATE_QUATERNION','ATTITUDE','LOCAL_POSITION_NED']:last[ty]=(time.monotonic(),z.to_dict())
  elif ty=='STATUSTEXT':events.append(z.to_dict())
def cmd(s,seconds=1.5):
 start=len(shell);b=(s+'\n').encode();assert len(b)<=70
 m.mav.serial_control_send(10,6,0,0,len(b),list(b)+[0]*(70-len(b)));poll(seconds);m.mav.serial_control_send(10,6,0,0,0,[0]*70);poll(.3);out=''.join(shell[start:]);print(s+'\n'+out,flush=True);return out
def check_stationary():
 h=last.get('HEARTBEAT');q=last.get('HIL_STATE_QUATERNION')
 if not h or not q or time.monotonic()-h[0]>2 or time.monotonic()-q[0]>.5:raise RuntimeError('Fresh autopilot heartbeat and SIH truth required')
 if h[1]['base_mode']&128:raise RuntimeError('Vehicle is armed; no changes applied')
 if math.sqrt(sum(q[1][k]**2 for k in ['vx','vy','vz']))>10:raise RuntimeError('SIH not stationary; no changes applied')
try:
 poll(3);check_stationary()
 before=cmd('param show MBE_ENABLE');hitl=cmd('param show SYS_HITL')
 assert re.search(r'MBE_ENABLE\s+\[[^]]+\]\s*:\s*1\b',before),before
 assert re.search(r'SYS_HITL\s+\[[^]]+\]\s*:\s*2\b',hitl),hitl
 (p.parent/'rollback/sih_mag_bias_20261001/board_before.txt').write_text(before+hitl)
 check_stationary();cmd('param set MBE_ENABLE 0');after=cmd('param show MBE_ENABLE')
 assert re.search(r'MBE_ENABLE\s+\[[^]]+\]\s*:\s*0\b',after),after
 cmd('param save',2);check_stationary()
 print('Verified MBE_ENABLE=0; rebooting disarmed stationary SIH board',flush=True)
 b=b'reboot\n';m.mav.serial_control_send(10,6,0,0,len(b),list(b)+[0]*(70-len(b)))
 old_att=last.get('ATTITUDE',(0,{}))[1].get('time_boot_ms',0);start=time.monotonic();reconnected=False
 while time.monotonic()-start<40:
  poll(.5);a=last.get('ATTITUDE')
  if a and a[0]>start and a[1].get('time_boot_ms',old_att)<old_att:reconnected=True;break
 print('reboot telemetry detected',reconnected,flush=True)
finally:
 try:m.mav.serial_control_send(10,0,0,0,0,[0]*70)
 except Exception:pass
 m.close();(p/'fix_application.json').write_text(json.dumps({'shell':''.join(shell),'last':last,'events':events},indent=2));(p/'fix_application.txt').write_text(''.join(shell))
