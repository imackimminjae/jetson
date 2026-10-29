#!/usr/bin/env python3
"""Replay EKF or SIH truth poses for one attempt in Isaac, without controllers."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
import rosbag2_py


def latest_zero_session(sweeps, scenario):
    name=f'scenario{scenario}'
    for directory in sorted(sweeps.glob('manual_bias_*'),
                            key=lambda p:p.stat().st_mtime,reverse=True):
        try:
            plan=json.loads((directory/'plan.json').read_text())
            progress=json.loads((directory/'progress.json').read_text())
        except (OSError,ValueError):
            continue
        if plan.get('bias_degrees') != [0]:
            continue
        for job in plan.get('jobs',[]):
            if job.get('scenario') != name:
                continue
            recorded=progress.get('completed',{}).get(job['id'])
            if recorded and (Path(recorded)/'bag/metadata.yaml').is_file():
                return Path(recorded)
    raise ValueError(f'{name}: completed zero-bias recording not found in {sweeps}')


def load_attempt(session, attempt):
    summaries=list(session.glob('results/*/summary.csv'))
    if len(summaries)!=1:
        raise ValueError('Expected one results summary in the session')
    rows=list(csv.DictReader(summaries[0].open()))
    if not 1<=attempt<=len(rows):
        raise ValueError(f'Attempt must be in 1..{len(rows)}')
    row=rows[attempt-1]
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/knu_batch/status','/motive/vehicle/odom_map']))
    from std_msgs.msg import String
    poses=[];phases={}
    while reader.has_next():
        topic,raw,stamp=reader.read_next()
        if topic=='/knu_batch/status':
            status=json.loads(deserialize_message(raw,String).data)
            phases[status['sequence']]=status
        else:
            message=deserialize_message(raw,Odometry)
            poses.append((stamp*1e-9,message.pose.pose))
    phases=sorted(phases.values(),key=lambda s:s['phase_stamp_sec'])
    start=next(s['phase_stamp_sec'] for s in phases if s['token']==row['token'] and s['phase']=='running')
    complete=next(s['phase_stamp_sec'] for s in phases if s['token']==row['token'] and s['phase']=='coasting')
    stop=next(s['phase_stamp_sec'] for s in phases if s['phase_stamp_sec']>complete and s['phase']!='coasting')
    selected=[(stamp-start,pose) for stamp,pose in poses if start<=stamp<stop]
    if len(selected)<2 or any(not math.isfinite(value) for _,p in selected for value in (p.position.x,p.position.y,p.position.z,p.orientation.z)):
        raise ValueError('Recorded poses missing or invalid')
    return selected,dict(route=row['route'],attempt=attempt,outcome=row['outcome'],token=row['token'],
                         drive_sec=complete-start,coast_sec=stop-complete,total_sec=stop-start,
                         start_stamp_sec=start,stop_stamp_sec=stop,samples=len(selected),source='ekf')


def map_yaw_alignment(session, start):
    """Use the same fixed map rotation as this attempt, without fitting its path."""
    alignments=[]
    for line in (session/'controllers.log').read_text().splitlines():
        if 'Map alignment initialized once:' not in line:
            continue
        stamp=re.search(r'\[([0-9]+\.[0-9]+)\]',line)
        yaw=re.search(r'yaw_delta=([-0-9.]+)',line)
        if stamp and yaw and float(stamp[1])<=start:
            alignments.append((float(stamp[1]),float(yaw[1])))
    if not alignments:
        raise ValueError('Recorded map alignment is missing for this attempt')
    return max(alignments)[1]


def load_truth_data(path):
    """Read an SIH archive with explicit units; reject broken or reordered data."""
    import numpy as np
    with np.load(path,allow_pickle=False) as archive:
        if str(archive['message_type'])!='HIL_STATE_QUATERNION':
            raise ValueError('Truth archive must contain HIL_STATE_QUATERNION data')
        columns=[np.asarray(archive[name],dtype=float) for name in
                 ('stamp_sec','latitude_deg','longitude_deg','altitude_m','yaw_ned_rad')]
    if any(c.ndim!=1 for c in columns) or len({len(c) for c in columns})!=1:
        raise ValueError('Truth arrays must be equally sized one-dimensional arrays')
    data=np.column_stack(columns)
    if len(data)<2 or not np.isfinite(data).all() or np.any(np.diff(data[:,0])<=0):
        raise ValueError('Truth samples must be finite and strictly ordered by time')
    if np.any(abs(data[:,1])>90) or np.any(abs(data[:,2])>180):
        raise ValueError('Invalid truth latitude/longitude')
    return data


def geodetic_enu(latitude, longitude, altitude, origin):
    """WGS84 geodetic coordinates to local east/north/up in metres."""
    import numpy as np
    def ecef(lat,lon,height):
        lat=np.radians(lat);lon=np.radians(lon)
        eccentricity_squared=6.6943799901413165e-3
        radius=6378137.0/np.sqrt(1-eccentricity_squared*np.sin(lat)**2)
        return np.array([(radius+height)*np.cos(lat)*np.cos(lon),
                         (radius+height)*np.cos(lat)*np.sin(lon),
                         (radius*(1-eccentricity_squared)+height)*np.sin(lat)])
    delta=ecef(latitude,longitude,altitude)-ecef(*origin)[:,None]
    lat,lon=np.radians(origin[:2])
    rotation=np.array([[-np.sin(lon),np.cos(lon),0],
                       [-np.sin(lat)*np.cos(lon),-np.sin(lat)*np.sin(lon),np.cos(lat)],
                       [np.cos(lat)*np.cos(lon),np.cos(lat)*np.sin(lon),np.sin(lat)]])
    return (rotation@delta).T


def truth_poses(data, start, stop, anchor_elapsed, anchor_pose, yaw_delta):
    """Fixed initial translation/rotation only; position and body yaw stay independent."""
    import numpy as np
    if data[0,0]>start or data[-1,0]<stop:
        raise ValueError('SIH truth does not cover the full attempt and coast')
    left=max(0,int(np.searchsorted(data[:,0],start,side='right'))-1)
    right=min(len(data),int(np.searchsorted(data[:,0],stop))+1)
    covering=data[left:right]
    if len(covering)<2 or np.max(np.diff(covering[:,0]))>.25:
        raise ValueError('SIH truth has a gap longer than 0.25 seconds in this attempt')
    anchor_stamp=start+anchor_elapsed
    anchor=np.array([np.interp(anchor_stamp,data[:,0],data[:,i]) for i in (1,2,3)])
    # Replay raw SIH sample times, with an interpolated sample at the first
    # recorded pose so the starting translation is exact.
    stamps=np.concatenate(([anchor_stamp],data[(data[:,0]>anchor_stamp)&(data[:,0]<stop),0]))
    latitude=np.interp(stamps,data[:,0],data[:,1])
    longitude=np.interp(stamps,data[:,0],data[:,2])
    altitude=np.interp(stamps,data[:,0],data[:,3])
    enu=geodetic_enu(latitude,longitude,altitude,anchor)
    c,s=math.cos(yaw_delta),math.sin(yaw_delta)
    x=anchor_pose.position.x+c*enu[:,0]-s*enu[:,1]
    y=anchor_pose.position.y+s*enu[:,0]+c*enu[:,1]
    z=anchor_pose.position.z+enu[:,2]
    ned_yaw=np.interp(stamps,data[:,0],np.unwrap(data[:,4]))
    yaw=np.arctan2(np.sin(math.pi/2-ned_yaw+yaw_delta),
                   np.cos(math.pi/2-ned_yaw+yaw_delta))
    selected=[]
    for i,stamp in enumerate(stamps):
        pose=PoseStamped().pose
        pose.position.x=float(x[i]);pose.position.y=float(y[i]);pose.position.z=float(z[i])
        pose.orientation.z=float(yaw[i])  # Isaac's scalar yaw convention, not a quaternion.
        pose.orientation.w=0.
        selected.append((float(stamp-start),pose))
    return selected


def load_truth_attempt(session, recorded, info, truth_file=None):
    start,stop=info['start_stamp_sec'],info['stop_stamp_sec']
    if truth_file is not None:
        candidates=[truth_file]
    else:
        workspace=Path(__file__).resolve().parents[3]
        candidates=sorted((workspace/'parameter_sweeps/replay_truth').glob('*.npz'))
    for path in candidates:
        data=load_truth_data(path)
        if data[0,0]<=start and data[-1,0]>=stop:
            yaw_delta=map_yaw_alignment(session,start)
            poses=truth_poses(data,start,stop,recorded[0][0],recorded[0][1],yaw_delta)
            return poses,dict(info,source='sih_truth',truth_file=str(path),samples=len(poses),
                              alignment='Recorded map rotation; one initial position anchor; raw SIH motion and body yaw')
    raise ValueError('No SIH truth archive covers this attempt. Existing archives cover '
                     'scenarios 22-25; use --source ekf for older recordings, or provide --truth-file')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path,nargs='?')
    parser.add_argument('--latest-zero',type=Path,
                        help='Find the latest completed zero-bias session under this sweep directory')
    parser.add_argument('--scenario',type=int,help='Scenario number for --latest-zero')
    parser.add_argument('--attempt',type=int,default=1)
    parser.add_argument('--source',choices=('ekf','truth'),default='ekf',
                        help='Replay recorded EKF poses, or actual SIH position and body yaw')
    parser.add_argument('--truth-file',type=Path,
                        help='Explicit SIH .npz archive (otherwise discover parameter_sweeps/replay_truth)')
    parser.add_argument('--delay',type=float,default=10.)
    parser.add_argument('--inspect-only',action='store_true')
    args=parser.parse_args()
    if not math.isfinite(args.delay) or args.delay<0:
        parser.error('--delay must be finite and nonnegative')
    if args.truth_file is not None and args.source!='truth':
        parser.error('--truth-file requires --source truth')
    if args.latest_zero is not None:
        if args.session is not None or args.scenario is None or args.scenario<1:
            parser.error('--latest-zero requires --scenario and cannot be combined with a session path')
        args.session=latest_zero_session(args.latest_zero,args.scenario)
    elif args.session is None or args.scenario is not None:
        parser.error('Supply a session path, or --latest-zero with --scenario')
    print(f'Recorded session: {args.session}',flush=True)
    poses,info=load_attempt(args.session,args.attempt)
    if args.source=='truth':
        poses,info=load_truth_attempt(args.session,poses,info,args.truth_file)
    print(json.dumps(info,ensure_ascii=False),flush=True)
    if args.inspect_only:
        return
    rclpy.init();node=rclpy.create_node('recorded_pose_replay')
    publisher=node.create_publisher(PoseStamped,'/motive/vehicle/pose',10)
    def publish(pose):
        message=PoseStamped();message.header.stamp=node.get_clock().now().to_msg();message.header.frame_id='map'
        message.pose=pose  # Preserve recorded yaw-scalar orientation.z convention.
        publisher.publish(message)
    try:
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        conflicts=[p.node_name for p in node.get_publishers_info_by_topic('/motive/vehicle/pose') if p.node_name!=node.get_name()]
        controllers=set(node.get_node_names())&{'upper_planner_node','lower_tracking_mpc_node','knu_batch_runner'}
        if conflicts or controllers:
            raise RuntimeError(f'Pose/control publishers already running: {conflicts or sorted(controllers)}')
        if not any('ros2_subscriber' in s.node_name for s in node.get_subscriptions_info_by_topic('/motive/vehicle/pose')):
            raise RuntimeError('Isaac pose subscriber not found')
        print(f'Isaac pose subscriber ready; starting in {args.delay:g}s at recorded speed',flush=True)
        deadline=time.monotonic()+args.delay
        while time.monotonic()<deadline:
            publish(poses[0][1]);rclpy.spin_once(node,timeout_sec=.05)
        origin=time.monotonic();print('REPLAY START',flush=True);announced=False
        for elapsed,pose in poses:
            while rclpy.ok() and time.monotonic()-origin<elapsed:
                rclpy.spin_once(node,timeout_sec=min(.02,max(0,elapsed-(time.monotonic()-origin))))
            if not rclpy.ok():break
            publish(pose)
            if not announced and elapsed>=info['drive_sec']:
                print('Recorded completion reached; replaying coast to stationary',flush=True);announced=True
        if rclpy.ok():
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                publish(poses[-1][1]);rclpy.spin_once(node,timeout_sec=.05)
            print('REPLAY COMPLETE: one attempt including recorded stop',flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except (OSError,KeyError,ValueError,RuntimeError,StopIteration) as error:
        raise SystemExit(f'Replay unavailable: {error or "recorded attempt phases missing"}')
