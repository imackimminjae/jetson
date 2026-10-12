#!/usr/bin/env python3
"""Streaming quick review; no ROS nodes and no decoded grid copies."""
from pathlib import Path
from collections import Counter, defaultdict
from datetime import datetime
from zoneinfo import ZoneInfo
import csv, gzip, hashlib, json, math, os, shutil
os.environ.setdefault('MPLCONFIGDIR','/tmp/all60_review_mpl')
os.environ.setdefault('ROS_DOMAIN_ID','177')
os.environ.setdefault('ROS_LOCALHOST_ONLY','1')
import numpy as np
import yaml, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

ROOT=Path('/home/imac/ros2_ws'); OUT=Path(__file__).resolve().parent
B=ROOT/'drive_batches/20261009_070714_all_d8c3cf'
for n in ('tables','signals','figures','configs'): (OUT/n).mkdir(exist_ok=True)
def dump(p,v): p.write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def csvout(p,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    op=gzip.open if str(p).endswith('.gz') else open
    with op(p,'wt',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def stats(v):
    a=np.asarray(v,float);a=a[np.isfinite(a)]
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),maximum=float(a.max())) if len(a) else dict(n=0,median=None,p95=None,p99=None,maximum=None)
def rmse(v):return float(np.sqrt(np.mean(np.square(v)))) if len(v) else None
def length(xy): return float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()) if len(xy)>1 else 0.
def clock(t):return datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).isoformat(timespec='milliseconds')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

rows=list(csv.DictReader(next((B/'results').glob('*/summary.csv')).open()))
ph=[]; previous=None; od=[]; lower=[]; events=[]; timings=[]; bs=[]; grids=[]
counts=Counter(); receive=defaultdict(list)
reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(B/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
selected=['/knu_batch/status','/motive/vehicle/odom_map','/debug/lower_mpc_trace',
          '/debug/upper_branch_event','/debug/upper_stage_timing',
          '/debug/upper_interval_centering_state','/bev/occupancy_grid']
reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
types={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in selected}
while reader.has_next():
    topic,blob,tn=reader.read_next(); t=tn/1e9;counts[topic]+=1;receive[topic].append(t)
    if topic=='/bev/occupancy_grid':continue # Never decode/store grid payloads.
    m=deserialize_message(blob,types[topic])
    if topic=='/knu_batch/status':
        v=json.loads(m.data);key=(v['index'],v['phase'],v['sequence'])
        if key!=previous:ph.append(dict(received_t=t,**v));previous=key
    elif topic=='/motive/vehicle/odom_map':
        od.append([t,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.pose.pose.position.x,
                   m.pose.pose.position.y,m.pose.pose.orientation.z,math.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y)])
    elif topic=='/debug/lower_mpc_trace':lower.append(list(m.data))
    elif topic=='/debug/upper_interval_centering_state':bs.append(list(m.data))
    elif topic=='/debug/upper_stage_timing':timings.append(dict(received_t=t,**json.loads(m.data)))
    elif topic=='/debug/upper_branch_event':
        e=json.loads(m.data)
        events.append({k:e.get(k) for k in ('t_start','t_emit','plan_seq','path_revision','x','y','yaw_rad','wp0','wp1','horizon','valid_plan','reason','status')})
print('Streamed',dict(counts),flush=True)
od=np.asarray(od);lo=np.asarray(lower);bs=np.asarray(bs)
dump(OUT/'phases.json',ph)
upper_rows=[]
for i,x in enumerate(timings):
    own=timings[i+1].get('previous_report_ms') if i+1<len(timings) and timings[i+1]['cycle_seq']==x['cycle_seq']+1 else None
    upper_rows.append(dict(received_t=x['received_t'],t_start=x['t_start'],cycle_seq=x['cycle_seq'],
        outcome=x.get('outcome'),wall_ms=x['wall_ms'],cpu_ms=x.get('cpu_ms'),own_report_ms=own,
        body_plus_own_report_ms=x['wall_ms']+own if own is not None else None))
csvout(OUT/'signals/upper_cycles.csv.gz',upper_rows)
csvout(OUT/'signals/upper_events.csv.gz',events)
csvout(OUT/'signals/odom.csv.gz',[dict(zip(['bag_receive_epoch_s','header_epoch_s','x_map_m','y_map_m','yaw_scalar_rad','speed_mps'],x)) for x in od])
LOWER=['stamp_ros_s','x_map_m','y_map_m','yaw_scalar_rad','speed_mps','nearest_dense_index','lateral_error_m','body_heading_error_rad','first_command_rad','applied_steer_norm','qp_valid','build_to_solver_return_ms','mode','virtual_steer_norm','virtual_throttle','virtual_brake','mavlink_throttle_norm','mavlink_steering_norm','objective','estimated_effective_steering_rad','last_applied_command_rad','actuator_model_enabled','actuator_delay_s','actuator_tau_s','solver_only_ms','selected_command_rad','output_command_rad_equivalent','error_feedback_available','reference_curvature_per_m','delta_prev_rad','rear_offset_m','command_scale_rad']
csvout(OUT/'signals/lower_trace.csv.gz',[dict(zip(LOWER,x)) for x in lo])
csvout(OUT/'signals/B_state.csv.gz',[dict(zip(['stamp_ros_s','B_before_m','B_after_m','threshold_m','candidate_length_m','candidate_step','candidate_interval'],x[:7])) for x in bs])
attempts=[];tracks={}; rt={}; stages_fail=[]
for index,c in enumerate(rows,1):
    states=[p for p in ph if p['index']==index]
    start=next((p['phase_stamp_sec'] for p in states if p['phase']=='running'),None)
    end=next((p['phase_stamp_sec'] for p in states if p['phase']=='coasting'),None)
    stop=next((p['phase_stamp_sec'] for p in ph if p['index']==index+1),ph[-1]['phase_stamp_sec'])
    ref=next((p['phase_stamp_sec'] for p in states if p['phase']=='reference'),None)
    r=dict(c,index=index,attempt=int(c['token'].split(':')[1])%5+1,departed=start is not None)
    if start is None or end is None:attempts.append(r);continue
    m=(lo[:,0]>=start)&(lo[:,0]<end); l=lo[m];active=l[((l[:,10]>0)|(l[:,27]>0))&np.isin(l[:,12],[0,1,2,5])]
    ev=[e for e in events if start<=e['t_start']<end];bad=[e for e in ev if not e['valid_plan']]
    cyc=[x for x in upper_rows if start<=x['t_start']<end]
    oma=(od[:,0]>=start)&(od[:,0]<end); om=(od[:,0]>=start)&(od[:,0]<stop)
    first=next((e for e in events if ref<=e['t_start']<start),None)
    bfirst=bs[np.argmin(abs(bs[:,0]-first['t_emit']))] if first else None
    pairs=l[1:];dt=np.diff(l[:,0]);rates=abs(np.diff(l[:,26]))/np.maximum(dt,1e-9)
    r.update(start_epoch_s=start,end_epoch_s=end,start_kst=clock(start),end_kst=clock(end),
        running_duration_s=end-start,running_distance_m=length(od[oma,2:4]),
        upper_attempts=len(ev),upper_failed=len(bad),upper_failure_reasons=json.dumps(dict(Counter(e['reason'] for e in bad)),ensure_ascii=False),
        lower_trace_samples=len(l),tracking_samples=len(active),lateral_rmse_m=rmse(active[:,6]),
        lateral_abs_p95_m=float(np.quantile(abs(active[:,6]),.95)) if len(active) else None,
        lateral_abs_max_m=float(abs(active[:,6]).max()) if len(active) else None,
        heading_rmse_deg=rmse(np.degrees(active[:,7])),
        command_saturation_pct=float(np.mean(abs(active[:,26])>=.3-1e-5)*100) if len(active) else None,
        output_rate_exceed_pairs=int(np.sum((rates>1.0471975512+1e-4)&(dt>0))),
        lower_modes=json.dumps(dict(Counter(int(x) for x in l[:,12]))),
        first_B_before_m=float(bfirst[1]) if bfirst is not None else None,
        first_waypoint_pair=str([first['wp0'],first['wp1']]) if first else None,
        upper_cycle_runtime_max_ms=max((x['body_plus_own_report_ms'] for x in cyc if x['body_plus_own_report_ms'] is not None),default=None),
        upper_cycle_deadline_miss_n=sum(x['body_plus_own_report_ms']>500 for x in cyc if x['body_plus_own_report_ms'] is not None),
        stop_epoch_s=stop)
    attempts.append(r);tracks[index]=od[om];rt[index]=(l,active,cyc)
csvout(OUT/'tables/per_attempt.csv',attempts)

summary=[];runtime=[]
for n in range(1,13):
    a=[x for x in attempts if x['route']==f'scenario{n}'];indices=[x['index'] for x in a if x['departed']]
    active=np.vstack([rt[i][1] for i in indices]); all_lower=np.vstack([rt[i][0] for i in indices]);cyc=[x for i in indices for x in rt[i][2]]
    v=[x['body_plus_own_report_ms'] for x in cyc if x['body_plus_own_report_ms'] is not None]
    config=B/'routes'/f'scenario{n}.csv'
    route=np.array([[float(x['x']),float(x['y'])] for x in csv.DictReader(config.open())])
    summary.append(dict(scenario=n,scheduled=len(a),departed=sum(x['departed'] for x in a),
        completed=sum(x['outcome']=='completed' for x in a),off_route=sum(x['outcome']=='off_route' for x in a),
        stopped_10s=sum(x['outcome']=='stopped_10s' for x in a),system_skipped=sum(x['outcome']=='skipped_system_fault' for x in a),
        stationary_stops=sum(x['stop_status']=='stationary' for x in a),route_polyline_m=length(route),
        upper_attempts=sum(x['upper_attempts'] for x in a),upper_failed=sum(x['upper_failed'] for x in a),
        lateral_rmse_m=rmse(active[:,6]),heading_rmse_deg=rmse(np.degrees(active[:,7])),
        command_saturation_pct=float(np.mean(abs(active[:,26])>=.3-1e-5)*100),
        upper_runtime_p95_ms=stats(v)['p95'],upper_runtime_max_ms=stats(v)['maximum'],
        upper_deadline_miss_n=sum(x>500 for x in v),
        lower_called_qp_n=int(np.sum(np.isfinite(all_lower[:,24]))),lower_invalid_called_qp_n=int(np.sum(np.isfinite(all_lower[:,24])&(all_lower[:,10]==0)))))
    for label,data in [('upper_all_cycles_body_plus_report_ms',v),
        ('upper_accepted_cycles_body_plus_report_ms',[x['body_plus_own_report_ms'] for x in cyc if x['outcome']=='success' and x['body_plus_own_report_ms'] is not None]),
        ('lower_solver_only_ms',all_lower[:,24]),('lower_build_to_solver_return_ms',all_lower[:,11][np.isfinite(all_lower[:,24])])]:
        runtime.append(dict(scenario=n,measurement=label,**stats(data)))
csvout(OUT/'tables/per_scenario.csv',summary);csvout(OUT/'tables/runtime.csv',runtime)
dump(OUT/'summary.json',dict(run_id=B.name,outcomes=dict(Counter(r['outcome'] for r in rows)),
    departed=sum(x['departed'] for x in attempts),scheduled=len(rows),scenarios=summary,
    message_counts=dict(counts),session_result=json.loads((B/'session_result.json').read_text())))
for p in ('effective_config.yaml','manifest.json','session_result.json'):shutil.copy2(B/p,OUT/'configs'/p)
shutil.copytree(B/'routes',OUT/'configs/routes',dirs_exist_ok=True)
dump(OUT/'sources_sha256.json',{str(p.relative_to(ROOT)):sha(p) for p in B.rglob('*') if p.is_file()})
previous_runs={1:'20261008_182344_scenario1_0cefb5',2:'20261008_183952_scenario2_5524df',3:'20261008_185426_scenario3_72c4df'}
nowconfig=yaml.safe_load((B/'effective_config.yaml').read_text());differences={}
for n,name in previous_runs.items():
    old=yaml.safe_load((ROOT/'drive_batches'/name/'effective_config.yaml').read_text());delta=[]
    for node in ('px4_ekf_bridge','upper_planner_node','lower_tracking_mpc_node','px4_odom_map_bridge'):
        oldp=old[node]['ros__parameters'];newp=nowconfig[node]['ros__parameters']
        for key in sorted(oldp.keys()|newp.keys()):
            if oldp.get(key)!=newp.get(key):delta.append(dict(node=node,parameter=key,old=oldp.get(key),new=newp.get(key)))
    differences[str(n)]=delta
dump(OUT/'configuration_comparison.json',differences)

plt.rcParams.update({'font.size':8,'axes.grid':True,'grid.alpha':.2})
mask=np.load(ROOT/'followup_20261009/route_design/road_mask.npy')[::2,::2]
ext=json.loads((ROOT/'followup_20261009/route_design/road_mask.json').read_text())['ext']
shutil.copy2(ROOT/'followup_20261009/route_design/road_mask.json',OUT/'configs/road_mask.json')
np.savez_compressed(OUT/'configs/road_backdrop.npz',road_mask=mask)
colors=['#0072B2','#E69F00','#009E73','#CC79A7','#D55E00']
def axis_route(ax,n,attempt=None):
    route=np.array([[float(x['x']),float(x['y'])] for x in csv.DictReader((B/'routes'/f'scenario{n}.csv').open())])
    ax.imshow(mask,origin='lower',extent=ext,cmap='Greys',vmin=0,vmax=1,alpha=.25)
    ax.plot(route[:,0],route[:,1],'k--o',ms=2,lw=1,label='SD-map waypoints')
    limits=[route]
    for a in attempts:
        if a['route']!=f'scenario{n}' or (attempt and a['attempt']!=attempt):continue
        tr=tracks.get(a['index'])
        if tr is None:continue
        k=tr[:,0]<a['end_epoch_s'];color=colors[a['attempt']-1]
        ax.plot(tr[k,2],tr[k,3],color=color,lw=1,label=f"A{a['attempt']} {a['outcome']}")
        coast=tr[~k];ax.plot(coast[:,2],coast[:,3],':',color=color,lw=.8)
        ax.plot(tr[k][-1,2],tr[k][-1,3],'x',color=color,ms=6)
        ax.plot(float(a['stop_x']),float(a['stop_y']),'s',color=color,ms=3)
        limits.append(tr[:,2:4])
    ax.plot(*route[0],'g^',ms=7,label='Route start');ax.plot(*route[-1],'k*',ms=8,label='Goal')
    pts=np.vstack(limits);mn=pts.min(0)-15;mx=pts.max(0)+15;center=(mn+mx)/2;half=max(mx-mn)/2
    ax.set_xlim(center[0]-half,center[0]+half);ax.set_ylim(center[1]-half,center[1]+half);ax.set_aspect('equal');ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
    return ax
fig,axs=plt.subplots(3,4,figsize=(17,13))
for n,ax in enumerate(axs.flat,1):
    axis_route(ax,n);s=summary[n-1];ax.set_title(f"S{n}: goal approach {s['completed']}/5, off_route {s['off_route']}, stopped {s['stopped_10s']}")
fig.suptitle(B.name+' | all 60 attempts | solid=running, dotted=coasting, x=outcome, square=stop\nRoad backdrop: reconstructed design map, not independent road-containment truth',fontsize=11)
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'figures/all_60_routes.png',dpi=220);fig.savefig(OUT/'figures/all_60_routes.pdf');plt.close(fig)
with PdfPages(OUT/'figures/scenario_all_attempts.pdf') as pdf:
    for n in range(1,13):
        fig,axs=plt.subplots(2,3,figsize=(14,9));axis_route(axs.flat[0],n);axs.flat[0].set_title(f'S{n}: all 5 attempts');axs.flat[0].legend(fontsize=6)
        for j in range(1,6):
            axis_route(axs.flat[j],n,j);a=attempts[(n-1)*5+j-1]
            axs.flat[j].set_title(f"S{n} A{j}: {a['outcome']}\n{a['start_kst'][11:23]} ~ {a['end_kst'][11:23]}")
        fig.suptitle(B.name+' | solid=running, dotted=coast; x=outcome, square=stop | design road backdrop')
        fig.tight_layout(rect=(0,0,1,.95));pdf.savefig(fig);fig.savefig(OUT/f'figures/scenario{n}_all5.png',dpi=200);plt.close(fig)
print('Route figures complete',flush=True)
fig,axs=plt.subplots(2,2,figsize=(13,8));ns=np.arange(1,13)
axs[0,0].bar(ns,[s['completed'] for s in summary],label='goal approach');axs[0,0].bar(ns,[s['off_route'] for s in summary],bottom=[s['completed'] for s in summary],label='off_route');axs[0,0].bar(ns,[s['stopped_10s'] for s in summary],bottom=[s['completed']+s['off_route'] for s in summary],label='stopped_10s');axs[0,0].legend();axs[0,0].set_ylabel('Attempts')
axs[0,1].bar(ns,[s['lateral_rmse_m'] for s in summary]);axs[0,1].set_ylabel('Lower-path lateral RMSE [m]')
axs[1,0].bar(ns,[100*s['upper_failed']/s['upper_attempts'] for s in summary]);axs[1,0].set_ylabel('Upper failed plans [%]')
axs[1,1].bar(ns,[s['command_saturation_pct'] for s in summary]);axs[1,1].set_ylabel('Output command at 0.3 rad bound [%]')
for ax in axs.flat:ax.set_xticks(ns);ax.set_xlabel('Scenario')
fig.suptitle(B.name+' | running phase only; completed = goal approach');fig.tight_layout();fig.savefig(OUT/'figures/results_metrics.png',dpi=220);fig.savefig(OUT/'figures/results_metrics.pdf');plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(13,7))
for n in range(1,13):
    cy=[x['body_plus_own_report_ms'] for i in range((n-1)*5+1,n*5+1) for x in rt[i][2] if x['body_plus_own_report_ms'] is not None]
    axs[0].scatter(np.full(len(cy),n),cy,s=2,alpha=.3)
    ls=np.vstack([rt[i][0] for i in range((n-1)*5+1,n*5+1)]);v=ls[:,24];v=v[np.isfinite(v)];axs[1].scatter(np.full(len(v),n),v,s=2,alpha=.25)
axs[0].axhline(500,c='red',ls='--',label='Upper period 500 ms');axs[0].legend();axs[0].set_ylabel('Upper body + own report [ms]');axs[0].set_yscale('log');axs[1].set_ylabel('Lower solver only [ms]');axs[1].set_xlabel('Scenario');axs[1].set_yscale('log')
for ax in axs:ax.set_xticks(ns)
fig.suptitle(B.name+' | successful and failed upper cycles; finite lower solver calls');fig.tight_layout();fig.savefig(OUT/'figures/runtime.png',dpi=220);fig.savefig(OUT/'figures/runtime.pdf');plt.close(fig)
with PdfPages(OUT/'figures/state_error_command_histories.pdf') as pdf:
    for n in range(1,13):
        fig,axs=plt.subplots(4,1,figsize=(12,9),sharex=True)
        for a in attempts[(n-1)*5:n*5]:
            l,act,_=rt[a['index']];color=colors[a['attempt']-1];start=a['start_epoch_s']
            axs[0].plot(l[:,0]-start,l[:,4],color=color,lw=.7,label=f"A{a['attempt']}: {a['outcome']}")
            axs[1].plot(act[:,0]-start,act[:,6],color=color,lw=.7)
            axs[2].plot(act[:,0]-start,np.degrees(act[:,7]),color=color,lw=.7)
            axs[3].plot(l[:,0]-start,l[:,26],color=color,lw=.7)
        for ax,label in zip(axs,['Speed [m/s]','Lower lateral error [m]','Body heading error [deg]','Output command [rad-equivalent]']):ax.set_ylabel(label)
        axs[0].legend(ncol=3,fontsize=7);axs[3].set_xlabel('Seconds from running start');fig.suptitle(B.name+f' | S{n} | all five runs, running phase only; command is not measured wheel angle');fig.tight_layout();pdf.savefig(fig);plt.close(fig)
print('Finished',dict(Counter(r['outcome'] for r in rows)),flush=True)
