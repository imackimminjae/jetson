"""PX4 diagnostic shell queries and bounded logger control through an existing MAVProxy UDP output socket.
Never bind ROS UDP ports, start a node, change parameters or send actuator/arm commands.
Replies are read from MAVProxy's existing tlog. Run only one shell client at a time.
"""
import argparse,os,time,json
from pathlib import Path
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
GROUPS={
 'record_start':['logger on','logger status'],
 'record_stop':['logger off','logger status'],
 'capture':['logger on','logger status','listener actuator_armed 1','logger off','logger status','ls /fs/microsd/log'],
 'identity':['simulator_sih status','land_detector status','ver all','logger status','param show IMU_GYRO_RATEMAX','param show SIH_T_TAU'],
 'state':['listener vehicle_local_position_groundtruth 1','listener vehicle_land_detected 1','listener estimator_status_flags 1','listener estimator_gps_status 1','listener vehicle_local_position 1','listener actuator_armed 1','listener actuator_outputs_sim 1'],
 'off_status':['logger status'],
 'record_check':['logger status','ls /fs/microsd/log','cat /fs/microsd/etc/logging/logger_topics.txt'],
 'sd_status':['mount','df','ls /fs','ls /fs/microsd','ls /fs/microsd/etc/logging','logger status','dmesg'],
 'log_index':['ls -l /fs/microsd/log/sess124','listener vehicle_local_position_groundtruth 1','listener vehicle_land_detected 1','listener estimator_status_flags 1','listener estimator_gps_status 1','listener vehicle_local_position 1','listener actuator_outputs_sim 1'],
 'log_files':['ls -l /fs/microsd/log/2026-09-12','ls -l /fs/microsd/log/sess123','ls /fs/microsd/etc/logging','param show SDLOG_PROFILE','param show SDLOG_MODE','listener -h'],
 'logs':['ls /fs/microsd/log','logger help','listener help','land_detector status'],
}
p=argparse.ArgumentParser();p.add_argument('--proxy-port',type=int,required=True);p.add_argument('--group',choices=GROUPS,required=True);p.add_argument('--tlog',default='/home/imac/ros2_ws/mav.tlog');a=p.parse_args()
if a.proxy_port in [14540,14541,14550]:raise SystemExit('Refusing a consumer port; use the verified MAVProxy source port.')
root=Path(__file__).resolve().parent;started=time.time();m=mavutil.mavlink_connection(f'udpout:127.0.0.1:{a.proxy_port}',source_system=246,source_component=191,dialect='common')
flags=mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND|mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE
try:
 for cmd in GROUPS[a.group]:
  buf=(cmd+'\n').encode();assert len(buf)<=70
  m.mav.serial_control_send(10,flags,0,0,len(buf),list(buf)+[0]*(70-len(buf)));time.sleep(1.8)
  m.mav.serial_control_send(10,flags,0,0,0,[0]*70);time.sleep(.6)
finally:
 if a.group=='capture':
  buf=b'logger off\n';m.mav.serial_control_send(10,flags,0,0,len(buf),list(buf)+[0]*(70-len(buf)));time.sleep(.6)
 m.mav.serial_control_send(10,0,0,0,0,[0]*70);m.close()
time.sleep(.4)
f=mavutil.mavlink_connection(a.tlog);records=[]
while True:
 x=f.recv_match(type=['SERIAL_CONTROL','HIL_STATE_QUATERNION','HEARTBEAT','ESTIMATOR_STATUS','EXTENDED_SYS_STATE','STATUSTEXT'])
 if x is None:break
 if x._timestamp>=started:records.append({**x.to_dict(),'t':x._timestamp})
f.close();text=''.join(bytes(r['data'][:r['count']]).decode(errors='replace') for r in records if r['mavpackettype']=='SERIAL_CONTROL')
(root/f'{a.group}_responses.json').write_text(json.dumps({'started':started,'commands':GROUPS[a.group],'records':records},indent=2));(root/f'{a.group}_shell.txt').write_text(text);print(text or 'NO SERIAL_CONTROL RESPONSE: no claim about board state can be made.')
