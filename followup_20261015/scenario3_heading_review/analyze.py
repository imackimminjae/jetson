"""Offline steering and trajectory evaluation, with each attempt weighted equally."""
from pathlib import Path
from collections import Counter
import json,pickle,csv,math,hashlib,os
import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.signal import savgol_filter
import yaml
os.environ['MPLCONFIGDIR']='/tmp/codex_rate_stability_mpl'
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]

def rms(a):return float(np.sqrt(np.mean(np.asarray(a)**2))) if len(a) else None
def p95(a):return float(np.percentile(a,95)) if len(a) else None
def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi
def stat(a):
    a=np.asarray(a,dtype=float)
    return dict(min=float(a.min()),median=float(np.median(a)),max=float(a.max()),p95=float(np.percentile(a,95))) if len(a) else {}

def project(points,route):
    vec=np.diff(route,axis=0);length=np.linalg.norm(vec,axis=1);cum=np.r_[0,np.cumsum(length)]
    diff=points[:,None,:]-route[:-1][None,:,:]
    u=np.clip(np.sum(diff*vec[None,:,:],axis=2)/(length**2)[None,:],0,1)
    feet=route[:-1][None,:,:]+u[:,:,None]*vec[None,:,:]
    delta=points[:,None,:]-feet
    idx=np.argmin(np.sum(delta**2,axis=2),axis=1);ar=np.arange(len(points))
    progress=cum[idx]+u[ar,idx]*length[idx]
    signed=(vec[idx,0]*delta[ar,idx,1]-vec[idx,1]*delta[ar,idx,0])/length[idx]
    return progress,signed,cum

def steering_metrics(l,mask,max_angle):
    # Index17 is the steering command actually sent through MAVLink, normalised.
    # Sign inversion is removed to match the controller's convention. Index31
    # is its configured radians-per-normalised-command scale, not a sensor angle.
    t=l[:,0];steer=np.degrees(-l[:,17]*l[:,31]);dt=np.diff(t)
    pairs=mask[:-1]&mask[1:]&(dt>.015)&(dt<.25)
    changes=np.diff(steer)[pairs];rates=changes/dt[pairs]
    distance=np.sum(.5*(l[:-1,4]+l[1:,4])[pairs]*dt[pairs])
    active_time=float(np.sum(dt[pairs]))
    if not np.any(mask) or not active_time or distance<=0:
        return dict(samples=int(mask.sum()), active_sec=active_time, travel_m=float(distance))
    # Common100Hz held-command representation with0.5-second smoothing.
    rt=np.arange(t[0],t[-1],.01);index=np.clip(np.searchsorted(t,rt,side='right')-1,0,len(t)-1);command=steer[index]
    good=mask[index]
    jitter=command-uniform_filter1d(command,50,mode='nearest')
    increments=np.diff(command);threshold=.5
    signs=np.sign(increments);signs[np.abs(increments)<threshold]=0
    significant=signs[(good[:-1]&good[1:])&(signs!=0)]
    reversals=int(np.sum(significant[1:]!=significant[:-1])) if len(significant)>1 else 0
    return dict(samples=int(mask.sum()),active_sec=active_time,travel_m=float(distance),
        variation_deg_per_sec=float(np.sum(abs(changes))/active_time),
        variation_deg_per_100m=float(np.sum(abs(changes))/distance*100),
        rate_rms_degps=rms(rates),rate_p95_degps=p95(abs(rates)),rate_max_degps=float(np.max(abs(rates))) if len(rates) else None,
        rapid_residual_rms_deg=rms(jitter[good]),
        limit_ratio_pct=float(100*np.mean(abs(steer[mask])>=np.degrees(max_angle)*.98)),
        reversals_over_0_5deg=reversals,
        reversal_per_sec=reversals/active_time,
        command_abs_p95_deg=p95(abs(steer[mask])))

sets={'20261015_010154_scenario3_f9c4a7':{'label':'기준: P5Hz / 하위 헤딩 0.5','set':'baseline_5hz'},'20261015_010707_scenario3_331a1b':{'label':'참고: P2Hz / 하위 헤딩 0.5','set':'reference_2hz'},'20261015_024902_scenario3_3778ac':{'label':'이번: P5Hz / 하위 헤딩 0.3','set':'lower_heading_0_3'}}
reports=[];curves={};validation=[];sources={}

def episodes(l, wanted, start):
    inds=np.flatnonzero(np.isin(l[:,12],wanted))
    blocks=np.split(inds,np.flatnonzero(np.diff(inds)>1)+1)
    step=np.median(np.diff(l[:,0]))
    return [dict(start_sec=float(l[b[0],0]-start),duration_sec=float(l[b[-1],0]-l[b[0],0]+step),modes=sorted(set(l[b,12].astype(int).tolist())),speed_start_mps=float(l[b[0],4])) for b in blocks if len(b)]

def gapstat(times,start,end):
    tt=np.asarray(times);delta=np.diff(np.r_[start,tt,end])
    return dict(samples=len(tt),max_gap_sec=float(delta.max()),gaps_over_0_5sec=int(np.sum(delta>.5)),gaps=[dict(start_sec=float(a-start),duration_sec=float(b-a)) for a,b in zip(np.r_[start,tt],np.r_[tt,end]) if b-a>.5])

for p in sorted(OUT.glob('decoded*.pkl')):
    d=pickle.load(p.open('rb'));cfg=yaml.safe_load(d['config']);key=d['batch'];b=Path(d['session_dir']);D=d['data'];route=np.asarray(d['route']);phase=[v for t,v in D['/knu_batch/status']]
    up=cfg['upper_planner_node']['ros__parameters'];lo=cfg['lower_tracking_mpc_node']['ros__parameters'];rate=lo['lower_controller_rate_hz']
    meta=yaml.safe_load((b/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information'];counts={v['topic_metadata']['name']:v['message_count'] for v in meta['topics_with_message_count']}
    for topic,vals in D.items(): assert len(vals)==counts[topic],(key,topic,len(vals),counts[topic])
    for path in b.rglob('*'):
        if path.is_file():sources[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    attempts=[];curves[key]=[]
    for i,row in enumerate(d['summary'],1):
        start=next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='running')
        end=next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='coasting')
        l=np.array([v for t,v in D['/debug/lower_mpc_trace'] if start<=t<end]);od=np.array([v for v in D['/motive/vehicle/odom_map'] if start<=v[0]<end])
        ev=[v for t,v in D['/debug/upper_branch_event'] if start<=v['t_start']<end];cy=[v for t,v in D['/debug/upper_stage_timing'] if start<=v['t_start']<end]
        assert len(l)>100
        speed=np.hypot(od[:,5],od[:,6]);s,offset,cum=project(od[:,1:3],route);sl,el,_=project(l[:,1:3],route)
        moving=l[:,4]>1;valid=(l[:,10]>0)|(l[:,27]>0);mask=moving&valid
        cmd=steering_metrics(l,moving,lo['lower_max_steering_angle_rad']);segments={}
        for name,a,z in [('approach',5,35),('first_turn',35,80),('middle',80,105),('second_turn',105,145),('exit',145,155)]:
            segmask=moving&(sl>=a)&(sl<z);segments[name]=steering_metrics(l,segmask,lo['lower_max_steering_angle_rad']);mv=segmask&valid
            segments[name].update(lateral_rmse_m=rms(l[mv,6]),heading_rmse_deg=rms(np.degrees(l[mv,7])))
        tt=np.arange(od[0,0],od[-1,0],.05);yaw=np.unwrap(od[:,3]);yr=savgol_filter(np.interp(tt,od[:,0],yaw),9,2,deriv=1,delta=.05,mode='interp');v=np.interp(tt,od[:,0],speed);gyro=np.degrees(yr);ymask=v>1;ya=np.diff(gyro)/.05;yp=ymask[1:]&ymask[:-1]
        plan_jumps=[];prev=None
        for e in ev:
            if not e['valid_plan']:continue
            path=np.array(e['published_world'])
            if len(path)>1:
                if prev is not None:
                    di=np.linalg.norm(prev-np.array([e['x'],e['y']]),axis=1);k=min(int(np.argmin(di)),len(prev)-2)
                    old=np.arctan2(*(prev[k+1]-prev[k])[::-1]);new=np.arctan2(*(path[1]-path[0])[::-1]);plan_jumps.append(abs(np.degrees(wrap(new-old))))
                prev=path
        a=dict(attempt=i,outcome=row['outcome'],start=start,end=end,duration_sec=float(row['duration_sec']),running_sec=end-start,stop_status=row['stop_status'],end_progress_ratio=float(row['progress_ratio']),end_cross_track_m=float(row['cross_track_m']),moving_mean_speed_mps=float(np.mean(l[moving,4])),initial_pose=od[0,1:4].tolist(),steering=cmd,segments=segments,lateral_rmse_m=rms(l[mask,6]),heading_rmse_deg=rms(np.degrees(l[mask,7])),waypoint_polyline_offset_rms_m=rms(offset[speed>1]),waypoint_polyline_offset_p95_m=p95(abs(offset[speed>1])),yaw_rate_abs_p95_degps=p95(abs(gyro[ymask])),yaw_accel_rms_degps2=rms(ya[yp]),yaw_accel_abs_p95_degps2=p95(abs(ya[yp])),upper_attempts=len(ev),upper_failures=sum(not e['valid_plan'] for e in ev),upper_failure_reasons=dict(Counter(e['reason'] for e in ev if not e['valid_plan'])),fallback_modes=dict(Counter(int(x) for x in l[:,12])),moving_fallback_modes=dict(Counter(int(x) for x in l[moving,12])),non_tracking_episodes=episodes(l,[3,4],start),fallback_episodes=episodes(l,[1,2,5],start),upper_wall_ms=stat([c['wall_ms'] for c in cy]),upper_over_period=sum(c['wall_ms']>1000/up['upper_planner_rate_hz'] for c in cy),lower_solve_ms=stat(l[:,11]),lower_solve_over_period=int(np.sum(l[:,11]>1000/rate)),lower_start_interval_ms=stat(np.diff(l[:,0])*1000),upper_interval_ms=stat([c['start_interval_ms'] for c in cy if c['start_interval_ms'] is not None]),plan_first_heading_jump_deg=stat(plan_jumps),max_progress_m=float(s.max()))
        a['inputs']={topic:gapstat([t for t,v in D[topic] if start<=t<end],start,end) for topic in ['/bev/occupancy_grid','/knu_batch/run_permission','/planner/goal_reached'] if topic in D}
        a['inputs']['/motive/vehicle/odom_map']=gapstat(od[:,0],start,end)
        a['inputs']['/debug/lower_mpc_trace']=gapstat(l[:,0],start,end)
        a['grid_gap_max_s']=a['inputs']['/bev/occupancy_grid']['max_gap_sec']
        attempts.append(a);curves[key].append(dict(attempt=i,outcome=row['outcome'],s=s,offset=offset,odom=od,lower=l,lower_s=sl,events=ev,start=start,end=end))
        print(key,i,row['outcome'],'TV',round(cmd['variation_deg_per_100m'],1),'yawA',round(a['yaw_accel_rms_degps2'],2),'BEVgap',round(a['grid_gap_max_s'],3),'fails',a['upper_failures'],'modes',a['fallback_modes'],flush=True)
    # Full-route repeatability uses completed runs only; failures are retained in all metrics and separate plots.
    complete=[c for c in curves[key] if c['outcome']=='completed'];sg=np.arange(5,155,.5)
    assert all(c['s'].max()>=sg[-1] for c in complete)
    xy=[];st=[];lat=[];heading=[]
    for c in complete:
        ss=np.maximum.accumulate(c['s']);uniq,idx=np.unique(ss,return_index=True)
        xy.append(np.column_stack([np.interp(sg,uniq,c['odom'][idx,k]) for k in [1,2]]))
        ls=np.maximum.accumulate(c['lower_s']);u,j=np.unique(ls,return_index=True)
        st.append(np.interp(sg,u,np.degrees(-c['lower'][j,17]*c['lower'][j,31])));lat.append(np.interp(sg,u,c['lower'][j,6]));heading.append(np.interp(sg,u,np.degrees(c['lower'][j,7])))
    xy=np.asarray(xy);spread=np.sqrt(np.var(xy[:,:,0],axis=0)+np.var(xy[:,:,1],axis=0))
    spatial=dict(completed_attempts=[c['attempt'] for c in complete],common_progress_min_m=float(sg[0]),common_progress_max_m=float(sg[-1]),rms_spread_m=rms(spread),p95_spread_m=p95(spread),max_spread_m=float(spread.max()),first_turn_rms_spread_m=rms(spread[(sg>=35)&(sg<80)]),second_turn_rms_spread_m=rms(spread[(sg>=105)&(sg<145)]),exit_rms_spread_m=rms(spread[sg>=145]))
    np.savez_compressed(OUT/f'spatial_{key}.npz',s=sg,xy=xy,mean_xy=xy.mean(axis=0),spread=spread,steering=st,lateral=lat,heading=heading)
    reports.append(dict(id=key,label=sets[key]['label'],set=sets[key]['set'],params=sets[key],attempts=attempts,spatial=spatial))
    validation.append(dict(session=key,selected_topic_counts_match_metadata=True,effective_config_snapshot_used=True,attempts=len(attempts)))

(OUT/'metrics.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n')
(OUT/'curves.pkl').write_bytes(pickle.dumps(curves))
(OUT/'sources_sha256.json').write_text(json.dumps(sources,indent=2)+'\n')
(OUT/'validation.json').write_text(json.dumps(dict(sessions=validation,source_hashes_saved=True,operating_code_changed=False),ensure_ascii=False,indent=2)+'\n')
rows=[];summaries=[]
for r in reports:
    for a in r['attempts']:rows.append(dict(set=r['set'],attempt=a['attempt'],outcome=a['outcome'],duration_sec=a['duration_sec'],speed_mps=a['moving_mean_speed_mps'],**a['steering'],lateral_rmse_m=a['lateral_rmse_m'],heading_rmse_deg=a['heading_rmse_deg'],yaw_accel_rms_degps2=a['yaw_accel_rms_degps2'],upper_failures=a['upper_failures'],grid_gap_max_s=a['grid_gap_max_s']))
    entry=dict(id=r['id'],set=r['set'],successes=sum(a['outcome']=='completed' for a in r['attempts']),runs=len(r['attempts']),completed_repeatability=r['spatial'])
    for population,aa in [('all',r['attempts']),('completed',[a for a in r['attempts'] if a['outcome']=='completed'])]:
        e={name:stat([a['steering'][name] for a in aa]) for name in ['variation_deg_per_100m','rate_p95_degps','rapid_residual_rms_deg','limit_ratio_pct']}
        e.update({name:stat([a[name] for a in aa]) for name in ['duration_sec','moving_mean_speed_mps','lateral_rmse_m','heading_rmse_deg','yaw_accel_rms_degps2','grid_gap_max_s']})
        e['upper_failures_total']=sum(a['upper_failures'] for a in aa);e['upper_attempts_total']=sum(a['upper_attempts'] for a in aa)
        entry[population]=e
    summaries.append(entry)
with (OUT/'per_attempt.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
(OUT/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
