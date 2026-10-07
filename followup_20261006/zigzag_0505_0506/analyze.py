"""Read-only review of saved scenario3 bags and preserved MAVLink extracts."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json, pickle, sqlite3, hashlib, re, sys
import numpy as np
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]; KST=ZoneInfo('Asia/Seoul')
sys.path.insert(0,str(ROOT/'followup_20261006/scripts'))
from analyze_runs import project
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
TAGS=['013717','034428','050536','050627']
SEG=[('straight',0,88),('J1',88,125),('J2',125,150),('exit',150,190)]
manifest={}; series={}; summaries={}; details={}; cached={}
def track(p):
    manifest[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
def clock(t): return datetime.fromtimestamp(float(t),KST).strftime('%H:%M:%S.%f')[:-3]
def q95(a): return float(np.quantile(np.abs(a),.95)) if len(a) else None
def decode(folder):
    out={}; classes={}
    for p in sorted((folder/'bag').glob('*.db3')):
        track(p); c=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True)
        for name,kind,t,blob in c.execute('select topics.name,topics.type,messages.timestamp,messages.data from messages join topics on topics.id=messages.topic_id order by messages.timestamp'):
            if name not in ['/motive/vehicle/odom_map','/debug/lower_mpc_trace','/debug/upper_branch_event','/debug/upper_interval_centering_state','/planner/goal_reached']: continue
            if kind not in classes: classes[kind]=get_message(kind)
            m=deserialize_message(blob,classes[kind]);t=t/1e9
            if kind=='nav_msgs/msg/Odometry': value=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z]
            elif kind=='std_msgs/msg/String': value=json.loads(m.data)
            elif kind=='std_msgs/msg/Float64MultiArray': value=list(m.data)
            else: value=m.data
            out.setdefault(name,[]).append((t,value))
        c.close()
    return out
def nearest(p,poly):
    v=np.diff(poly,axis=0); n=np.sum(v*v,axis=1); u=np.clip(np.sum((p-poly[:-1])*v,axis=1)/np.maximum(n,1e-12),0,1)
    q=poly[:-1]+u[:,None]*v; dd=np.linalg.norm(p-q,axis=1);j=np.argmin(dd)
    if (j==0 and u[j]<=1e-6) or (j==len(v)-1 and u[j]>=1-1e-6): return None
    return float(np.cross(v[j],p-q[j])/max(1e-9,np.linalg.norm(v[j])))
logs=Path('/home/imac/.ros/log')
upper=list(logs.glob('upper_planner_node_*.log'))
logtime=lambda p:int(p.stem.split('_')[-1])/1000
for tag in TAGS:
    folder=ROOT/f'drive_debug_20261006_{tag}_light'; cfgp=folder/'config_snapshot.yaml'; track(cfgp)
    cfg=yaml.safe_load(cfgp.read_text()); data=decode(folder)
    od=data['/motive/vehicle/odom_map']; ot=np.array([x[0] for x in od]); oxy=np.array([x[1] for x in od]); prog,_=project(GP,oxy[:,0],oxy[:,1])
    bagstart=ot[0]; ul=max((p for p in upper if logtime(p)<=bagstart+1),key=logtime); track(ul); U=ul.read_text()
    start=float(re.findall(r'\[(\d+\.\d+)\]',U)[0]); end=float(re.findall(r'\[(\d+\.\d+)\]',U)[-1])
    tlp=ROOT/'followup_20261006'/({'013717':'drive_0137/tlog_0137.pkl','034428':'runs_0342_0358/tlog_0340.pkl'}.get(tag,'zigzag_0505_0506/tlog.pkl'))
    if str(tlp) not in cached:
        track(tlp); raw=pickle.load(tlp.open('rb')); cached[str(tlp)]=raw.get('data',raw)
    T=cached[str(tlp)]
    cl=np.array([[m['_t'],m['param1'],-m['param2']*.35] for m in T['COMMAND_LONG'] if m['command']==187 and start<=m['_t']<=end])
    hs=np.array([[m['_t'],np.hypot(m['vx'],m['vy'])/100,-m['yawspeed']] for m in T['HIL_STATE_QUATERNION'] if start<=m['_t']<=end])
    on=cl[cl[:,1]>0][0,0]; off=cl[cl[:,1]>0][-1,0]
    tr=np.array([x[1] for x in data['/debug/lower_mpc_trace']]); pt,_=project(GP,tr[:,1],tr[:,2])
    # All telemetry statistics below are clipped to bag coverage, avoiding endpoint extrapolation.
    aa=max(on,ot[0],tr[0,0]); zz=min(off,ot[-1],tr[-1,0]); grid=np.arange(aa,zz,.1)
    cmd=np.interp(grid,cl[:,0],cl[:,2]); ph=np.interp(grid,ot,prog); vv=np.interp(grid,hs[:,0],hs[:,1]); yaw=np.degrees(np.interp(grid,hs[:,0],hs[:,2]))
    bb,ab=butter(2,.3/5,'high'); hp=np.degrees(filtfilt(bb,ab,cmd))
    goals=[(float(t),float(v)) for t,v in re.findall(r'\[(\d+\.\d+)\] \[upper_planner_node\]: goal reached: remaining=[0-9.]+ direct=([0-9.]+)',U)]
    fails=[float(t) for t in re.findall(r'\[(\d+\.\d+)\] \[upper_planner_node\]: egocentric MIQP failed',U)]
    events=[x[1] for x in data['/debug/upper_branch_event']]
    evrows=[]; prev=None
    for e in events:
        p=float(project(GP,np.array([e['x']]),np.array([e['y']]))[0][0]); pl=np.array(e['planned_world']); ref=np.array(e['preview_world'])
        row={'t':e['t_start'],'time':clock(e['t_start']),'progress':p,'xy':[e['x'],e['y']], 'valid':e['valid_plan'],'reason':e['reason'],'horizon':e['horizon'],'counts':e['candidate_counts'],'selected':e['selected_corridors'], 'reference_correction_m':[], 'previous_plan_lateral_m':[]}
        if e['valid_plan'] and len(pl)>2:
            for k in [1,2]:
                tangent=ref[k]-ref[k-1];row['reference_correction_m'].append(float(np.cross(tangent,pl[k]-ref[k])/max(1e-9,np.linalg.norm(tangent))))
                row['previous_plan_lateral_m'].append(nearest(pl[k],prev[1]) if prev and 0<e['t_start']-prev[0]<1.5 else None)
            row['plan_heading_deg']=np.degrees(np.arctan2(np.diff(pl,axis=0)[:,1],np.diff(pl,axis=0)[:,0])).tolist()
            prev=(e['t_start'],pl)
        evrows.append(row)
    s={'bag':str(folder),'upper_log':str(ul),'first_throttle':clock(on),'bag_start':clock(ot[0]),'bag_start_lag_s':float(ot[0]-on),'rate_hz':cfg['upper_planner_node']['ros__parameters']['upper_planner_rate_hz'],'r_dpsi':cfg['upper_planner_node']['ros__parameters']['upper_r_dpsi'], 'rd':cfg['lower_tracking_mpc_node']['ros__parameters']['lower_rd_steering_rate'],'goal_direct_m':goals[0][1] if goals else None, 'upper_failures_all':[clock(t) for t in fails], 'upper_failures_in_bag_progress':[float(np.interp(t,ot,prog)) for t in fails if ot[0]<=t<=ot[-1]], 'max_command_gap_s':float(max(np.diff(cl[(cl[:,0]>=on)&(cl[:,0]<=off),0]))), 'max_lower_trace_gap_s':float(max(np.diff(tr[:,0]))), 'lower_modes':{str(int(k)):int(np.sum(tr[:,12]==k)) for k in np.unique(tr[:,12])},'segments':{}}
    for sn,a,b in SEG:
        m=(ph>=a)&(ph<b)&(vv>1); mt=(pt>=a)&(pt<b)&(tr[:,10]==1)&(tr[:,0]>=aa)&(tr[:,0]<=zz)
        er=[e for e in evrows if a<=e['progress']<b and e['valid']]
        jumps=[abs(e['previous_plan_lateral_m'][0]) for e in er if e['previous_plan_lateral_m'] and e['previous_plan_lateral_m'][0] is not None]
        if not np.any(m):continue
        s['segments'][sn]={'duration_s':float(m.sum()*.1),'sat_fraction':float(np.mean(abs(cmd[m])>=.298)), 'steering_highpass_rms_deg':float(np.sqrt(np.mean(hp[m]**2))),'yawrate_abs_p95_deg_s':q95(yaw[m]),'speed_median_m_s':float(np.median(vv[m])),'local_path_lateral_p95_m':q95(tr[mt,6]),'local_path_heading_p95_deg':q95(np.degrees(tr[mt,7])), 'k1_previous_plan_shift_abs_p95_m':q95(jumps),'shift_sample_count':len(jumps)}
    s['J1']=next(('right' if x>103 else 'left wrong' for x,y in oxy[:,:2] if y>-38),'unconfirmed')
    s['J2']='E' if np.any((oxy[:,0]>140)&(oxy[:,1]<8)&(oxy[:,1]>-12)) else ('N wrong' if np.any((oxy[:,1]>15)&(oxy[:,0]<135)) else 'unconfirmed')
    summaries[tag]=s; details[tag]=evrows; series[tag]={'oxy':oxy,'prog':prog,'t':grid,'p':ph,'cmd':np.degrees(cmd),'yawrate':yaw,'events':events,'lower':tr,'lower_progress':pt,'hs':hs,'cl':cl,'on':on,'off':off,'od_t':ot,'centering':data['/debug/upper_interval_centering_state']}
    print(tag,json.dumps(s,ensure_ascii=False))
(HERE/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
(HERE/'plan_events.json').write_text(json.dumps(details,ensure_ascii=False,indent=2)+'\n')
(HERE/'inputs_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
pickle.dump(series,(HERE/'series.pkl').open('wb'))
