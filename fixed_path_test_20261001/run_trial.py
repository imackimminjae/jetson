"""Run the existing MPC against the actual PX4 SIH plant on isolated ROS domain 91."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

os.environ['MAVLINK20']='1'
from pymavlink import mavutil

ROOT=Path('/home/imac/ros2_ws')
CONFIG=ROOT/'src/virtual_control/config/tracking_control_split.yaml'
BIN=ROOT/'install/virtual_control/lib/virtual_control'


def board_check():
    m=mavutil.mavlink_connection('udpin:127.0.0.1:14540',source_system=246,source_component=191,dialect='common')
    try:
        hb=m.wait_heartbeat(timeout=5)
        if hb is None or hb.get_srcSystem()!=1 or hb.get_srcComponent()!=1:
            raise RuntimeError('No PX4 heartbeat')
        if hb.base_mode & 128:
            raise RuntimeError('PX4 is already armed; another run may be active')
        params={};truth=None
        for name in ['SYS_HITL','MBE_ENABLE']:
            m.mav.param_request_read_send(1,1,name.encode(),-1)
        m.mav.command_long_send(1,1,511,0,115,33333,0,0,0,0,0)
        end=time.monotonic()+5
        while time.monotonic()<end:
            z=m.recv_match(blocking=True,timeout=.1)
            if z is None:continue
            if z.get_type()=='PARAM_VALUE':
                # PX4 PARAM_VALUE uses bytewise encoding for integer parameters.
                value = struct.unpack('<i',struct.pack('<f',z.param_value))[0] if z.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32 else z.param_value
                params[z.param_id]=value
            if z.get_type()=='HIL_STATE_QUATERNION':truth=z.to_dict()
            if all(n in params for n in ['SYS_HITL','MBE_ENABLE']) and truth:break
        if params.get('SYS_HITL')!=2 or params.get('MBE_ENABLE')!=0:
            raise RuntimeError('Required SIH/magnetometer settings not verified: '+str(params))
        if truth is None or math.sqrt(sum(truth[k]**2 for k in ['vx','vy','vz']))>10:
            raise RuntimeError('SIH truth is missing or moving')
        return {'heartbeat':hb.to_dict(),'parameters':params,'truth':truth}
    finally:m.close()


def finish_board(original_mode):
    m=mavutil.mavlink_connection('udpin:127.0.0.1:14540',source_system=246,source_component=191,dialect='common')
    try:
        if m.wait_heartbeat(timeout=5) is None:return {'error':'No heartbeat for shutdown verification'}
        m.mav.command_long_send(1,1,187,0,0,0,float('nan'),float('nan'),float('nan'),float('nan'),0)
        m.mav.command_long_send(1,1,400,0,0,0,0,0,0,0,0)
        end=time.monotonic()+25;last={}
        while time.monotonic()<end:
            z=m.recv_match(blocking=True,timeout=.2)
            if z is None:continue
            if z.get_type() in ['HEARTBEAT','HIL_STATE_QUATERNION','COMMAND_ACK']:last[z.get_type()]=z.to_dict()
            h=last.get('HEARTBEAT',{});v=last.get('HIL_STATE_QUATERNION',{})
            if h and not h['base_mode'] & 128 and v and math.sqrt(sum(v[k]**2 for k in ['vx','vy','vz']))<10:
                m.mav.set_mode_send(1,1,original_mode)
                last['verified_disarmed_stationary']=True
                return last
        last['verified_disarmed_stationary']=False
        return last
    finally:m.close()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--name',default='sih_fixed_004');ap.add_argument('--curvature',default=.04,type=float)
    ap.add_argument('--no-bag',action='store_true');ap.add_argument('--async-publish',action='store_true')
    a=ap.parse_args();out=ROOT/'fixed_path_test_20261001'/a.name;out.mkdir(exist_ok=False)
    # Refuse to compete with a normal controller or bridge for the MAVLink endpoints.
    for port in [14540,14541]:
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        try:s.bind(('0.0.0.0',port))
        finally:s.close()
    before=board_check();(out/'board_before.json').write_text(json.dumps(before,indent=2))
    (out/'config_snapshot.yaml').write_text(CONFIG.read_text())
    env=os.environ.copy();env.update(ROS_DOMAIN_ID='91',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros_logs'))
    if a.async_publish:env['RMW_FASTRTPS_PUBLICATION_MODE']='ASYNCHRONOUS'
    (out/'test_conditions.json').write_text(json.dumps({'ros_domain':91,'localhost_only':True,'async_publish':a.async_publish,'rosbag_enabled':not a.no_bag,'curvature':a.curvature},indent=2))
    processes={}
    def spawn(name,args):
        f=(out/(name+'.log')).open('wb')
        proc=subprocess.Popen(args,env=env,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        f.close();processes[name]=proc;return proc
    def stop(name):
        proc=processes.get(name)
        if proc and proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=2)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    def args(executable,*overrides):
        r=[str(BIN/executable),'--ros-args','--params-file',str(CONFIG)]
        for v in overrides:r+=['-p',v]
        return r
    try:
        spawn('bridge',args('px4_ekf_bridge.py','connection_url:=udpin:0.0.0.0:14541','output_pose_topic:=/px4/sih/odom','publish_yaw_scalar_pose:=true','configure_px4_parameters:=false'))
        spawn('map_bridge',args('px4_odom_map_bridge','route_name:=scenario3','output_topic:=/motive/vehicle/odom_map'))
        probe=spawn('probe',['/usr/bin/python3',str(ROOT/'src/virtual_control/scripts/fixed_path_probe.py'),'--output',str(out),'--curvature',str(a.curvature)])
        deadline=time.monotonic()+35
        while not (out/'READY').exists():
            if probe.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Probe not ready; inspect bridge/probe logs')
            time.sleep(.2)
        if not a.no_bag:
            spawn('bag',['ros2','bag','record','-o',str(out/'bag'),'/motive/vehicle/odom_map','/px4/ekf_odom','/px4_ekf_bridge/status','/debug/lower_mpc_trace','/control/applied_cmd','/fixed_path_probe/status','/rosout'])
        spawn('lower',args('lower_tracking_mpc_node','pixhawk_output_backend:=mavlink_udp','mavlink_enable:=true','mavlink_auto_manual_mode:=true','mavlink_auto_arm:=true','mavlink_force_arm:=false','mavlink_disarm_on_shutdown:=true','batch_supervision:=true'))
        time.sleep(2)
        if processes['lower'].poll() is not None:raise RuntimeError('Lower controller exited')
        (out/'RUN').write_text('User authorized lower-only SIH trial; immutable geometry, production gains.')
        print('RUNNING',out,flush=True)
        deadline=time.monotonic()+60
        while probe.poll() is None:
            if time.monotonic()>deadline:raise RuntimeError('Trial exceeded supervisor timeout')
            if processes['lower'].poll() is not None:raise RuntimeError('Lower controller exited during test')
            time.sleep(.2)
        print('RESULT', (out/'STOPPED').read_text() if (out/'STOPPED').exists() else 'probe error',flush=True)
    finally:
        # Stop permission publishing, then the controller. Its destructor sends
        # neutral + disarm; verify independently after it releases UDP 14540.
        stop('probe');stop('lower')
        after=finish_board(before['heartbeat']['custom_mode'])
        (out/'board_after.json').write_text(json.dumps(after,indent=2))
        for name in ['bag','map_bridge','bridge']:stop(name)
        print('BOARD_AFTER',after,flush=True)


if __name__=='__main__':main()
