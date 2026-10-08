#!/usr/bin/env python3
"""Read-only HIL evidence extraction. No ROS nodes, publishers, hardware or tuning.
Run after sourcing ROS Humble and this workspace. Grids are streamed, never decoded to disk.
Existing telemetry is checked against source hashes and every recorded odom/lower sample.
"""
from pathlib import Path
from collections import Counter, defaultdict, deque
from datetime import datetime
from zoneinfo import ZoneInfo
import csv, gzip, hashlib, json, os, shutil, sys
os.environ.setdefault('ROS_DOMAIN_ID', '177')
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path('/home/imac/ros2_ws')
OUT=Path(__file__).resolve().parent
REVIEW=ROOT/'followup_20261008/batches_182344_183952_185426_review'
RUNS={1:'20261008_182344_scenario1_0cefb5',2:'20261008_183952_scenario2_5524df',3:'20261008_185426_scenario3_72c4df'}
for name in ('configs','tables','signals','scenes','figures','sources'):
    (OUT/name).mkdir(exist_ok=True)

def stamp(t): return datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).isoformat(timespec='milliseconds')
def dump(p,obj): p.write_text(json.dumps(obj,indent=2,ensure_ascii=False,allow_nan=True)+'\n')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()
def csvout(p,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for row in rows for k in row))
    op=gzip.open if str(p).endswith('.gz') else open
    with op(p,'wt',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def jlines(p,rows):
    with gzip.open(p,'wt') as f:
        for row in rows:f.write(json.dumps(row,ensure_ascii=False)+'\n')
def stats(v):
    v=np.asarray([x for x in v if x is not None],float);v=v[np.isfinite(v)]
    return dict(n=len(v),median=float(np.median(v)),p95=float(np.quantile(v,.95)),p99=float(np.quantile(v,.99)),maximum=float(v.max())) if len(v) else dict(n=0,median=None,p95=None,p99=None,maximum=None)
def rms(v):return float(np.sqrt(np.mean(np.asarray(v)**2))) if len(v) else None
def length(p):return float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()) if len(p)>1 else 0.
def scope(t,attempts):
    for a in attempts:
        if a['start']<=t<a['end']:return a['attempt'],'running'
        if a['reference_time']<=t<a['start']:return a['attempt'],'reference'
        if a['end']<=t<a['stop_time']:return a['attempt'],'coasting'
    return 0,'setup_or_other'

LOWER=['stamp_ros_s','x_map_m','y_map_m','yaw_scalar_rad','speed_mps','nearest_dense_index',
 'lateral_error_m','body_heading_error_rad','mpc_first_command_rad','applied_steer_norm','qp_valid',
 'build_to_solver_return_ms','fallback_mode','virtual_steer_norm','virtual_throttle','virtual_brake',
 'mavlink_throttle_norm','mavlink_steering_norm','objective','estimated_effective_steering_rad',
 'last_applied_command_rad','actuator_model_enabled','actuator_delay_s','actuator_tau_s',
 'solver_only_ms','selected_command_rad','output_command_rad_equivalent','error_feedback_available',
 'reference_curvature_per_m','delta_prev_rate_reference_rad','point_to_rear_axle_m','command_scale_rad']
ODOM=['x_map_m','y_map_m','yaw_scalar_rad','vx_map_mps','vy_map_mps','header_ros_s','frame','z_map_m']

def scene_specs(s,attempts,events):
    requests={
      1:[('branch',6,[(110,-38),(111,-24),(119,-9),(141,-3)]),
         ('wrong_exit',9,[(111,-27),(115,-17),(122,-3),(129,11)]),
         ('sway',1,[(109,-37),(111,-25),(114,-20),(117,-13)])],
      2:[('branch',1,[(-223,91),(-209,82),(-199,73),(-182,67)])],
      3:[('branch',1,[(226,152),(240,141),(254,139),(265,152)]),
         ('curve',1,[(187,198),(191,181),(201,169)])]}[s]
    scenes=[]
    for group,attempt,points in requests:
        a=attempts[attempt-1];ev=[e for e in events if a['start']<=e['t_start']<a['end']]
        for j,p in enumerate(points):
            candidates=ev
            if group=='branch' and j==1:
                multi=[e for e in ev if e['max_candidates']>=2 and np.hypot(e['x']-p[0],e['y']-p[1])<18]
                if multi:candidates=multi
            e=min(candidates,key=lambda e:np.hypot(e['x']-p[0],e['y']-p[1]))
            scenes.append(dict(scenario=s,attempt=attempt,group=group,panel=j+1,event=e))
    failure_attempt={1:1,2:1,3:4}[s]
    a=attempts[failure_attempt-1];ev=[e for e in events if a['start']<=e['t_start']<a['end']]
    bad=next(e for e in ev if not e['valid_plan']);t=bad['t_start']
    times=[t-1,t,t+1,t+3] if s!=2 else [t-1,t,t+2,a['end']-.5]
    for j,tt in enumerate(times):
        e=min(ev,key=lambda e:abs(e['t_start']-tt))
        scenes.append(dict(scenario=s,attempt=failure_attempt,group='failure',panel=j+1,event=e))
    for z in scenes:
        z['id']=f"s{s}_a{z['attempt']:02d}_{z['group']}_{z['panel']}"
        z['run_id']=RUNS[s];z['timestamp_kst']=stamp(z['event']['t_start'])
    return scenes

def extract_extras(s,d,summary,scenes):
    """One raw bag pass: hashes, sample validation, path records, small scene grids, mosaic."""
    run=ROOT/'drive_batches'/RUNS[s];attempts=summary['attempts']
    od=d['/motive/vehicle/odom_map'];ot=np.array([t for t,v in od]);oxy=np.array([v[:3] for t,v in od]);oxy[:,2]=np.unwrap(oxy[:,2])
    allxy=oxy[:,:2];base=np.floor((allxy.min(axis=0)-35)/.5)*.5;top=np.ceil((allxy.max(axis=0)+35)/.5)*.5
    shape=np.ceil((top-base)/.5).astype(int)[::-1]
    votes=np.zeros(shape,np.uint32);road=np.zeros(shape,np.uint32)
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types()}
    counts=Counter();idx=Counter();grid_meta=Counter();grid_values=Counter();params=[];paths=[];grid_times=[];grid_stamps=[];recent=deque(maxlen=12);last_mosaic=-1e20;scene_by_seq=defaultdict(list);checks=Counter();mosaic_n=0;zero_stamp=0
    for z in scenes:scene_by_seq[z['event']['plan_seq']].append(z)
    while reader.has_next():
        name,blob,tn=reader.read_next();t=tn*1e-9;counts[name]+=1
        if name not in ('/bev/occupancy_grid','/parameter_events','/planner/lower_reference_path/with_arclength','/debug/upper_branch_event','/debug/lower_mpc_trace','/motive/vehicle/odom_map'):continue
        m=deserialize_message(blob,types[name])
        if name=='/bev/occupancy_grid':
            g=m.info;h=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            meta=(g.width,g.height,g.resolution,g.origin.position.x,g.origin.position.y,m.header.frame_id)
            grid_meta[meta]+=1;grid_times.append(t);grid_stamps.append(h);zero_stamp+=h==0
            arr=np.asarray(m.data,dtype=np.int8).reshape(g.height,g.width)
            if counts[name]==1:grid_values.update({int(k):int(v) for k,v in zip(*np.unique(arr,return_counts=True))})
            recent.append((t,h,meta,arr))
            attempt,phase=scope(t,attempts)
            if phase not in ('running','coasting') or t-last_mosaic<.75:continue
            j=int(np.searchsorted(ot,t));
            if j==0 or j==len(ot) or ot[j]-ot[j-1]>.2:continue
            f=(t-ot[j-1])/(ot[j]-ot[j-1]);pose=oxy[j-1]*(1-f)+oxy[j]*f
            yy,xx=np.mgrid[0:g.height:2,0:g.width:2];bx=g.origin.position.x+(xx+.5)*g.resolution;by=g.origin.position.y+(yy+.5)*g.resolution
            c,si=np.cos(pose[2]),np.sin(pose[2]);wx=pose[0]+c*bx-si*by;wy=pose[1]+si*bx+c*by
            ix=np.floor((wx-base[0])/.5).astype(int);iy=np.floor((wy-base[1])/.5).astype(int);aa=arr[::2,::2]
            valid=(aa>=0)&(ix>=0)&(iy>=0)&(ix<shape[1])&(iy<shape[0]);np.add.at(votes,(iy[valid],ix[valid]),1);np.add.at(road,(iy[valid],ix[valid]),(aa[valid]<=1).astype(np.uint32));last_mosaic=t;mosaic_n+=1
        elif name=='/debug/upper_branch_event':
            e=json.loads(m.data)
            for z in scene_by_seq.get(e['plan_seq'],[]):
                candidates=[g for g in recent if g[0]<=e['t_start']]
                if not candidates:continue
                gt,gh,meta,arr=candidates[-1];p=OUT/'scenes'/f"{z['id']}_grid.npz"
                np.savez_compressed(p,grid=arr)
                z.update(grid_file=str(p.relative_to(OUT)),grid_receive_s=gt,grid_header_s=gh,grid_metadata=list(meta),grid_receive_age_at_plan_s=e['t_start']-gt,grid_pose_policy='planner pose; latest bag-received grid before cycle; exact subscriber snapshot not recorded')
        elif name=='/planner/lower_reference_path/with_arclength':
            p=m.path;paths.append(dict(receive_s=t,header_s=p.header.stamp.sec+p.header.stamp.nanosec*1e-9,frame=p.header.frame_id,xy=[[x.pose.position.x,x.pose.position.y] for x in p.poses],s=list(m.s)))
        elif name=='/parameter_events':
            vals={}
            for p in list(m.new_parameters)+list(m.changed_parameters):
                v=p.value;attr={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}.get(v.type)
                if attr:
                    vv=getattr(v,attr);vals[p.name]=list(vv) if v.type>=5 else vv
            params.append(dict(receive_s=t,node=m.node,values=vals))
        else:
            i=idx[name];cached=d[name][i];idx[name]+=1
            assert abs(cached[0]-t)<2e-6
            if name=='/debug/lower_mpc_trace':actual=list(m.data);expected=cached[1]
            else:
                actual=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,m.twist.twist.linear.x,m.twist.twist.linear.y,m.header.stamp.sec+m.header.stamp.nanosec/1e9,m.pose.pose.position.z]
                expected=cached[1][:6]+[cached[1][7]];assert cached[1][6]==m.header.frame_id
            assert np.allclose(actual,expected,atol=0,rtol=0,equal_nan=True);checks[name]+=1
    for n,c in summary['counts'].items():assert counts[n]==c,(n,counts[n],c)
    for z in scenes:
        e=z['event'];tt=e['t_emit']+.002
        p=max((p for p in paths if p['receive_s']<=tt),key=lambda p:p['receive_s'],default=None)
        # Prefer the actual published lower path from this cycle, using its generation stamp.
        matched=[p for p in paths if e['t_start']-.001<=p['header_s']<=e['t_emit']+.001]
        if e['valid_plan'] and matched:p=min(matched,key=lambda p:abs(p['header_s']-e['t_start']))
        z['lower_path']=p
    np.savez_compressed(OUT/'scenes'/f's{s}_road_mosaic.npz',votes=votes,road_votes=road,origin=base,resolution=.5)
    jlines(OUT/'signals'/f's{s}_lower_paths.jsonl.gz',paths);dump(OUT/'configs'/f's{s}_recorded_parameter_events.json',params)
    csvout(OUT/'signals'/f's{s}_grid_times.csv.gz',[dict(receive_s=t,header_s=h) for t,h in zip(grid_times,grid_stamps)])
    result=dict(topic_counts=counts,raw_sample_checks=checks,grid_metadata=[dict(values=list(k),count=v) for k,v in grid_meta.items()],first_grid_values=grid_values,grid_zero_stamp_count=zero_stamp,grid_count=len(grid_times),mosaic_frames=mosaic_n,mosaic_sample_interval_s=.75,mosaic_pixel_stride=2,mosaic_resolution_m=.5,grid_receive_interval_s=stats(np.diff(grid_times)),lower_paths=len(paths),bev_source_timing_recorded='/debug/bev_source_timing' in counts)
    dump(OUT/'tables'/f's{s}_input_inventory.json',result)
    return result

trial_rows=[];runtime_rows=[];failure_rows=[];compliance_rows=[];scoped_rows=[];scene_rows=[];sources=[];aggregate=[];all_validations=[]
for s,runid in RUNS.items():
    print('Processing',runid,flush=True)
    run=ROOT/'drive_batches'/runid;review=REVIEW/f'scenario{s}'
    summary=json.loads((review/'summary.json').read_text());attempts=summary['attempts'];d=json.load(gzip.open(review/'telemetry.json.gz','rt'))['data']
    expected=json.loads((review/'sources_sha256.json').read_text())
    for path,value in expected.items():
        p=Path(path);assert sha(p)==value, f'Source changed: {p}'
        sources.append(dict(scenario=s,run_id=runid,file=str(p),bytes=p.stat().st_size,sha256=value,role='original log/config'))
    for name in ('effective_config.yaml','manifest.json','session_result.json'):
        shutil.copy2(run/name,OUT/'configs'/f's{s}_{name}')
    ev=[dict(receive_s=t,**e) for t,e in d['/debug/upper_branch_event']]
    tm=[dict(receive_s=t,**e) for t,e in d['/debug/upper_stage_timing']]
    lo=np.asarray([v for t,v in d['/debug/lower_mpc_trace']],float)
    ot=np.asarray([t for t,v in d['/motive/vehicle/odom_map']]);od=np.asarray([v[:6]+[v[7]] for t,v in d['/motive/vehicle/odom_map']],float)
    route=np.array(summary['route']);ss=scene_specs(s,attempts,ev);inventory=extract_extras(s,d,summary,ss);scene_rows.extend(ss)
    # Direct recorded signals; masks and derived statistics are separate tables.
    lower_records=[]
    for (rx,v) in d['/debug/lower_mpc_trace']:
        a,ph=scope(v[0],attempts);lower_records.append(dict(run_id=runid,attempt=a,phase=ph,receive_s=rx,**dict(zip(LOWER,v))))
    csvout(OUT/'signals'/f's{s}_lower_trace.csv.gz',lower_records)
    csvout(OUT/'signals'/f's{s}_odometry.csv.gz',[dict(run_id=runid,attempt=scope(t,attempts)[0],phase=scope(t,attempts)[1],receive_s=t,**dict(zip(ODOM,v))) for t,v in d['/motive/vehicle/odom_map']])
    jlines(OUT/'signals'/f's{s}_upper_events.jsonl.gz',ev);jlines(OUT/'signals'/f's{s}_upper_stage_timing.jsonl.gz',tm)
    jlines(OUT/'signals'/f's{s}_batch_status.jsonl.gz',[dict(receive_s=t,**v) for t,v in d['/knu_batch/status']])
    csvout(OUT/'signals'/f's{s}_waypoints.csv',[dict(index=i,x_map_m=x,y_map_m=y) for i,(x,y) in enumerate(route)])
    csvout(OUT/'signals'/f's{s}_B.csv.gz',[dict(receive_s=t,**dict(zip(['stamp_ros_s','B_before_m','B_m','threshold_m','candidate_m','candidate_step','candidate_interval'],v))) for t,v in d['/debug/upper_interval_centering_state']])
    csvout(OUT/'signals'/f's{s}_rosout.csv.gz',[dict(receive_s=t,**v) for t,v in d['/rosout']])
    # Timings include successful, failed, and pre-solver returns, with scope explicitly retained.
    augmented=[]
    for i,t in enumerate(tm):
        z=dict(t);z['own_report_ms']=tm[i+1]['previous_report_ms'] if i+1<len(tm) and tm[i+1]['cycle_seq']==t['cycle_seq']+1 else None
        z['cycle_including_report_ms']=t['wall_ms']+z['own_report_ms'] if z['own_report_ms'] is not None else None
        z['solve_full_call_ms']=sum(x['wall_ms'] for x in t['stages'] if x['name']=='solve_upper_full_call') if any(x['name']=='solve_upper_full_call' for x in t['stages']) else None
        a,ph=scope(t['t_start'],attempts);z.update(attempt=a,phase=ph);augmented.append(z)
    csvout(OUT/'tables'/f's{s}_cycle_times.csv.gz',[{k:v for k,v in z.items() if k!='stages'} for z in augmented])
    for scope_name in ('running','entire_bag'):
        e=[e for e in ev if scope_name=='entire_bag' or scope(e['t_start'],attempts)[1]=='running']
        times=[t for t in augmented if scope_name=='entire_bag' or t['phase']=='running']
        m=np.ones(len(lo),bool) if scope_name=='entire_bag' else np.array([scope(t,attempts)[1]=='running' for t in lo[:,0]])
        l=lo[m];ec=Counter(x['reason'] for x in e);modes=Counter(int(x) for x in l[:,12])
        scoped_rows.append(dict(scenario=s,run_id=runid,scope=scope_name,upper_attempts=len(e),upper_published=sum(x['valid_plan'] for x in e),upper_solver_called=sum(x['solver_called'] for x in e),upper_solver_rejected=ec['solver_rejected'],upper_no_candidate=ec['no_candidate'],upper_preview_failed=ec['preview_failed'],upper_retained_path_events=sum(x['retained_previous_path'] for x in e),upper_side_guard_events=sum(x['side_boundary_tail_start']>=2 for x in e),upper_nominal_interval_cycles=sum(any(row[8]==1 for row in x['intervals_world']) for x in e),upper_nominal_selected_cycles=sum(any(row[8]==1 and len(x['selected_corridors'])>int(row[0]) and x['selected_corridors'][int(row[0])]==row[1] for row in x['intervals_world']) for x in e),lower_trace_samples=len(l),lower_solver_calls=int(np.isfinite(l[:,24]).sum()),lower_solver_failed=int((np.isfinite(l[:,24])&(l[:,10]!=1)).sum()),**{f'lower_mode_{i}_samples':modes[i] for i in range(6)}))
        metrics=[('upper_solver_all_called',[x['solver_time_ms'] for x in e if x['solver_called']],500),('upper_solver_success',[x['solver_time_ms'] for x in e if x['valid_plan'] and x['solver_called']],500),('upper_full_solve_call',[x['solve_full_call_ms'] for x in times],500),('upper_cycle_excluding_own_report',[x['wall_ms'] for x in times],500),('upper_cycle_including_report',[x['cycle_including_report_ms'] for x in times],500),('upper_cycle_success_including_report',[x['cycle_including_report_ms'] for x in times if x['outcome']=='success'],500),('upper_start_interval',[x['start_interval_ms'] for x in times],500),('lower_solver_all_called',l[np.isfinite(l[:,24]),24],100),('lower_solver_success',l[(l[:,10]==1)&np.isfinite(l[:,24]),24],100),('lower_build_to_solver_return_all_called',l[np.isfinite(l[:,24]),11],100),('lower_build_to_solver_return_success',l[(l[:,10]==1)&np.isfinite(l[:,24]),11],100)]
        for metric,values,period in metrics:
            q=stats(values);finite=np.array([v for v in values if v is not None and np.isfinite(v)])
            is_cycle=metric in ('upper_cycle_including_report','upper_cycle_excluding_own_report','upper_cycle_success_including_report')
            runtime_rows.append(dict(scenario=s,run_id=runid,scope=scope_name,metric=metric,unit='ms',period_ms=period,**q,max_scope_slack_ms=period-q['maximum'] if q['n'] else None,above_period_count=int((finite>period).sum()),above_period_percent=float(np.mean(finite>period)*100) if len(finite) else None,deadline_interpretation='measured upper cycle body; excludes callback queue delay' if is_cycle else 'not full cycle deadline statistic'))
    for a in attempts:
        idx=a['attempt'];lm=lo[(lo[:,0]>=a['start'])&(lo[:,0]<a['end'])];active=lm[((lm[:,10]==1)|(lm[:,27]==1))&np.isin(lm[:,12],[0,1,2,5])];moving=active[active[:,4]>1]
        es=[e for e in ev if a['start']<=e['t_start']<a['end']]
        mask=(ot>=a['start'])&(ot<a['end']);mask_all=(ot>=a['start'])&(ot<a['stop_time']);xy=od[mask,:2];xyall=od[mask_all,:2]
        # First-step rate constraint uses controller's recorded delta_prev and configured dt.
        cap=1.0471975512*.1;rate_excess=np.abs(active[:,25]-active[:,29])-cap
        sequential=np.diff(lm[:,26]);dt=np.diff(lm[:,0]);usable=(dt>0)&(dt<.3)
        seq_rates=np.abs(sequential[usable]/dt[usable]);output_jumps=(seq_rates>1.0471975512+1e-4)
        bresult='intended_exit_observed' if a['outcome']=='completed' else ('wrong_exit_observed' if s==1 else 'route_failure; inspect trajectory; no automatic exit-ID truth')
        row=dict(scenario=s,run_id=runid,attempt=idx,runner_outcome=a['outcome'],branch_result=bresult,goal_approach=int(a['outcome']=='completed'),target_stop_success='not demonstrated',running_start_ros_s=a['start'],running_end_ros_s=a['end'],start_kst=stamp(a['start']),end_kst=stamp(a['end']),running_duration_s=a['end']-a['start'],runner_setup_inclusive_duration_s=float(a['duration_sec']),route_polyline_length_m=length(route),observed_distance_running_m=length(xy),observed_distance_through_coast_m=length(xyall),start_x_m=float(xy[0,0]),start_y_m=float(xy[0,1]),decision_x_m=float(xy[-1,0]),decision_y_m=float(xy[-1,1]),goal_x_m=float(route[-1,0]),goal_y_m=float(route[-1,1]),stop_x_m=float(a['stop_x']),stop_y_m=float(a['stop_y']),final_stop_distance_to_goal_m=float(a['stop_goal_distance_m']),upper_attempts=len(es),upper_failures=sum(not e['valid_plan'] for e in es),lower_error_valid_n=len(active),moving_error_valid_n=len(moving),lateral_rmse_m=rms(active[:,6]),lateral_max_abs_m=float(np.max(abs(active[:,6]))) if len(active) else None,body_heading_rmse_rad=rms(active[:,7]),body_heading_max_abs_rad=float(np.max(abs(active[:,7]))) if len(active) else None,lateral_rmse_m_speed_gt1=rms(moving[:,6]),heading_rmse_rad_speed_gt1=rms(moving[:,7]),output_command_at_limit_n=int((np.abs(active[:,26])>=.3-1e-5).sum()),output_command_at_limit_percent=float(np.mean(abs(active[:,26])>=.3-1e-5)*100) if len(active) else None,selected_command_rate_constraint_exceeded_n=int((rate_excess>1e-5).sum()),selected_command_rate_constraint_max_excess_rad=float(max(0,rate_excess.max())) if len(active) else None,consecutive_output_rate_gt_limit_n=int(output_jumps.sum()),consecutive_output_rate_test_n=len(seq_rates),max_consecutive_output_rate_radps=float(max(seq_rates)) if len(seq_rates) else None,coast_seconds=a['terminal_neutral']['seconds_to_phase_end'] if a['terminal_neutral'] else None,neutral_to_stop_displacement_m=a['terminal_neutral']['displacement_to_stop_m'] if a['terminal_neutral'] else None)
        trial_rows.append(row)
        failure_rows.append(dict(scenario=s,run_id=runid,attempt=idx,phase='running_end',t_ros_s=a['end'],timestamp_kst=stamp(a['end']),x_m=row['decision_x_m'],y_m=row['decision_y_m'],event='runner_'+a['outcome'],reason=a.get('detail',''),retained_previous_path='',status='goal approach only' if a['outcome']=='completed' else 'runner termination criterion'))
    for e in ev:
        a,ph=scope(e['t_start'],attempts)
        if not e['valid_plan']:
            failure_rows.append(dict(scenario=s,run_id=runid,attempt=a,phase=ph,t_ros_s=e['t_start'],timestamp_kst=stamp(e['t_start']),x_m=e['x'],y_m=e['y'],event='upper_failure',reason=e['reason'],retained_previous_path=e['retained_previous_path'],retained_path_age_s=e['retained_path_age_sec'],status=e['status'],plan_seq=e['plan_seq'],solver_called=e['solver_called']))
        if ph!='running' or not e['valid_plan']:continue
        p=np.asarray(e['planned_world']);selected=e['selected_corridors']
        for row in e['intervals_world']:
            k,j=int(row[0]),int(row[1])
            if len(selected)<=k or selected[k]!=j or len(p)<=k:continue
            skipped=e['side_boundary_tail_start']>=2 and k>=e['side_boundary_tail_start']
            aa=np.array(row[2:4]);bb=np.array(row[4:6]);axis=bb-aa;w=np.linalg.norm(axis)
            if w<1e-12:continue
            axis/=w;z=float(np.dot(p[k]-aa,axis));margin=min(z,w-z)
            compliance_rows.append(dict(scenario=s,run_id=runid,attempt=a,t_ros_s=e['t_start'],plan_seq=e['plan_seq'],k=k,candidate=j,source_mode=int(row[8]),guard_constraint_skipped=skipped,published_node=k<len(e['published_world']),lateral_slab_margin_m=margin,violation_m=max(0.,-margin),exceeds_1mm_tolerance=bool(margin<-.001 and not skipped)))
    # Lower mode transitions, including goal and supervisor stop (both mode 4).
    previous=None
    for z in lo:
        mode=int(z[12]);a,ph=scope(z[0],attempts)
        if mode!=previous:
            failure_rows.append(dict(scenario=s,run_id=runid,attempt=a,phase=ph,t_ros_s=z[0],timestamp_kst=stamp(z[0]),x_m=z[1],y_m=z[2],event='lower_mode_transition',reason=f'{previous}->{mode}',status={0:'QP command',1:'previous sequence',2:'hold',3:'neutral invalid/stale',4:'goal or batch permission neutral',5:'error feedback fallback'}[mode]));previous=mode
    rows=[r for r in trial_rows if r['scenario']==s];errmask=np.array([scope(t,attempts)[1]=='running' for t in lo[:,0]])&((lo[:,10]==1)|(lo[:,27]==1))&np.isin(lo[:,12],[0,1,2,5]);errs=lo[errmask]
    outcome=Counter(a['outcome'] for a in attempts)
    aggregate.append(dict(scenario=s,run_id=runid,trials=len(attempts),goal_approach=outcome['completed'],off_route=outcome['off_route'],stopped_10s=outcome['stopped_10s'],other_termination=len(attempts)-outcome['completed']-outcome['off_route']-outcome['stopped_10s'],route_length_m=length(route),lower_error_valid_samples=len(errs),lateral_rmse_m=rms(errs[:,6]),lateral_max_abs_m=float(max(abs(errs[:,6]))),body_heading_rmse_deg=float(np.degrees(rms(errs[:,7]))),body_heading_max_abs_deg=float(np.degrees(max(abs(errs[:,7])))),output_command_at_limit_percent=float(np.mean(abs(errs[:,26])>=.3-1e-5)*100),upper_attempts=sum(r['upper_attempts'] for r in rows),upper_failures=sum(r['upper_failures'] for r in rows)))
    all_validations.append(dict(scenario=s,raw_checks=inventory['raw_sample_checks'],source_hashes_verified=len(expected),grid_count=inventory['grid_count']))
    print('Done',s,'scenes',len(ss),'running upper',aggregate[-1]['upper_attempts'],flush=True)

csvout(OUT/'tables/trials.csv',trial_rows);csvout(OUT/'tables/scenario_summary.csv',aggregate)
csvout(OUT/'tables/runtime_statistics.csv',runtime_rows);csvout(OUT/'tables/event_counts_by_scope.csv',scoped_rows)
csvout(OUT/'tables/failure_and_mode_events.csv',sorted(failure_rows,key=lambda r:(r['scenario'],r['t_ros_s'])))
csvout(OUT/'tables/plan_interval_checks.csv.gz',compliance_rows);csvout(OUT/'sources/original_files.csv',sources)
dump(OUT/'scenes/scene_manifest.json',scene_rows);dump(OUT/'tables/extraction_validation.json',all_validations)
dump(OUT/'tables/plan_interval_summary.json',dict(tolerance_m=.001,method='project planned node onto selected processed lateral interval axis; longitudinal displacement unconstrained; skip guarded tail',by_scenario={s:dict(nodes=sum(r['scenario']==s for r in compliance_rows),guarded_skipped=sum(r['scenario']==s and r['guard_constraint_skipped'] for r in compliance_rows),checked=sum(r['scenario']==s and not r['guard_constraint_skipped'] for r in compliance_rows),violations=sum(r['scenario']==s and r['exceeds_1mm_tolerance'] for r in compliance_rows),max_violation_m=max([r['violation_m'] for r in compliance_rows if r['scenario']==s and not r['guard_constraint_skipped']],default=0)) for s in RUNS}))
print('Evidence extraction complete',flush=True)
