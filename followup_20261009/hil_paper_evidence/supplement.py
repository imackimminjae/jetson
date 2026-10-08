#!/usr/bin/env python3
"""Additional audits, historic input-loss context, and measured runtime overrun."""
from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import csv,gzip,hashlib,json,os,shutil,subprocess
os.environ.setdefault('MPLCONFIGDIR','/tmp/hil_paper_matplotlib')
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
plt.rcParams.update({'font.size':9,'savefig.dpi':300,'pdf.fonttype':42})
def writecsv(path,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def dump(p,j):p.write_text(json.dumps(j,indent=2,ensure_ascii=False)+'\n')
def clock(t):return datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

# Full applied settings, corroboration and intentionally limited current-host inventory.
settings=[];checks=[]
for s in (1,2,3):
    c=yaml.safe_load((OUT/f'configs/s{s}_effective_config.yaml').read_text());events=json.loads((OUT/f'configs/s{s}_recorded_parameter_events.json').read_text());recorded={}
    for e in events:recorded.setdefault(e['node'].lstrip('/'),{}).update(e['values'])
    for node,block in c.items():
        for key,val in block['ros__parameters'].items():
            observed=recorded.get(node,{})
            settings.append(dict(scenario=s,node=node,parameter=key,applied_launch_value=json.dumps(val,ensure_ascii=False),recorded_parameter_value=json.dumps(observed[key]) if key in observed else '',evidence='effective_config + parameter_events' if key in observed else 'effective_config; partial parameter recording'))
            if key in observed:checks.append(dict(scenario=s,node=node,parameter=key,equal=val==observed[key]))
    for node,observed in recorded.items():
        for k in ('wheelbase','vehicle_length_m','use_sim_time'):
            if k in observed and k not in c.get(node,{}).get('ros__parameters',{}):settings.append(dict(scenario=s,node=node,parameter=k,applied_launch_value='',recorded_parameter_value=json.dumps(observed[k]),evidence='parameter_events (default resolved at run)'))
writecsv(OUT/'tables/applied_settings.csv',settings);dump(OUT/'tables/config_crosschecks.json',checks)
assert all(z['equal'] for z in checks)
host={p:Path(p).read_text(errors='replace').replace('\x00','') for p in ['/etc/os-release','/etc/nv_tegra_release','/proc/device-tree/model','/proc/cpuinfo','/proc/meminfo'] if Path(p).exists()}
host['uname']=subprocess.check_output(['uname','-a'],text=True).strip();host['package_versions']=subprocess.check_output(['dpkg-query','-W','ros-humble-rclcpp','ros-humble-rosbag2-storage-mcap'],text=True);host['scope']='Current analysis host inventory, not an archived per-run hardware snapshot'
dump(OUT/'configs/current_host_inventory.json',host)
selected_sources=['README_HILS.md','src/virtual_control/src/tracking_control.cpp','src/virtual_control/src/lower_tracking_mpc_node.cpp','src/virtual_control/src/px4_odom_map_bridge_node.cpp','src/virtual_control/scripts/px4_ekf_bridge.py','src/virtual_control/scripts/knu_batch_monitor.py','src/virtual_control/scripts/knu_batch_runner.py','src/virtual_control/include/virtual_control/planner_stage_timer.hpp','src/virtual_control/docs/upper_stage_timing.md','src/imac_interfaces/msg/PathWithArcLength.msg','src/mpclib_vendor/mpclib_vendor/third_party/miqplib/src/daqp/CMakeLists.txt','followup_20261006/interval_consistency/bev_source/rgb_to_occupancy.original.cpp','followup_20261006/interval_consistency/bev_source/DIAGNOSTICS_README.txt','px4_rest_audit_20261002/usb_identity_shell.txt','px4_followup_20261003/identity_shell.txt','followup_20261006/HANDOFF_20261006_branch_steering.txt']
inventory=[]
for rel in selected_sources:
    src=ROOT/rel;dest=OUT/'sources/code_and_prior_config'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,dest);inventory.append(dict(original=str(src),copy=str(dest.relative_to(OUT)),sha256=sha(src),scope='current source or dated prior evidence; not all are per-run snapshots'))
writecsv(OUT/'sources/code_source_inventory.csv',inventory)
manifest=json.loads((OUT/'configs/s3_manifest.json').read_text());binaries=[]
# The original manifest retains the authoritative run hashes. Compare any existing files.
def visit(x):
    if isinstance(x,dict):
        for k,v in x.items():
            if str(k).startswith('/home/imac/') and isinstance(v,str) and len(v)==64:
                p=Path(k);binaries.append(dict(file=k,run_sha256=v,current_sha256=sha(p) if p.is_file() else '',match=(sha(p)==v) if p.is_file() else False))
            else:visit(v)
    elif isinstance(x,list):
        for v in x:visit(v)
visit(manifest);writecsv(OUT/'sources/run_hash_vs_current.csv',binaries)

# Inventory every completed/interrupted batch currently in drive_batches, without pooling cohorts.
batchrows=[];allresults=[]
for run in sorted((ROOT/'drive_batches').iterdir()):
    if not (run/'effective_config.yaml').exists():continue
    c=yaml.safe_load((run/'effective_config.yaml').read_text());u=c.get('upper_planner_node',{}).get('ros__parameters',{});result_files=list((run/'results').glob('*/summary.csv'));rows=[]
    for p in result_files:
        with p.open() as f:rows.extend(list(csv.DictReader(f)))
    result=json.loads((run/'session_result.json').read_text()) if (run/'session_result.json').exists() else {}
    counts=Counter(r['outcome'] for r in rows)
    batchrows.append(dict(run_id=run.name,primary=int(run.name in [f'{x}' for x in ['20261008_182344_scenario1_0cefb5','20261008_183952_scenario2_5524df','20261008_185426_scenario3_72c4df']]),status=result.get('status',{}).get('phase'),recorded_results=len(rows),outcomes=json.dumps(counts),waypoint_bias_enabled=u.get('enable_waypoint_bias'),configured_yaw_bias_rad=u.get('waypoint_yaw_bias_rad'),applied_yaw_bias_rad=u.get('waypoint_yaw_bias_rad',0) if u.get('enable_waypoint_bias') else 0,source=str(run),note='Do not equate result-row count with actual driven attempts; initial setup failures included'))
    for i,r in enumerate(rows):allresults.append(dict(run_id=run.name,result_row=i+1,**r))
writecsv(OUT/'tables/all_recorded_batches_inventory.csv',batchrows);writecsv(OUT/'tables/all_recorded_batch_result_rows.csv',allresults)

# Output rate audit: distinguish the QP's reference constraint from successive issued commands.
rate=[]
for s in (1,2,3):
    rows=list(csv.DictReader(gzip.open(OUT/f'signals/s{s}_lower_trace.csv.gz','rt')))
    for prev,cur in zip(rows,rows[1:]):
        if prev['phase']!='running' or cur['phase']!='running' or prev['attempt']!=cur['attempt']:continue
        dt=float(cur['stamp_ros_s'])-float(prev['stamp_ros_s']);a=float(prev['output_command_rad_equivalent']);b=float(cur['output_command_rad_equivalent']);delta=abs(b-a)
        if not 0<dt<.3 or delta/dt<=1.0471975512+1e-4:continue
        cat='within_nominal_dt_step; elapsed interval jitter' if delta<=1.0471975512*.1+1e-5 else ('controller_rate_reference_changed' if cur['qp_valid']=='1.0' and abs(float(cur['delta_prev_rate_reference_rad'])-a)>1e-5 else 'neutral/mode change or other')
        rate.append(dict(scenario=s,run_id=cur['run_id'],attempt=cur['attempt'],t_ros_s=cur['stamp_ros_s'],time_kst=clock(float(cur['stamp_ros_s'])),dt_s=dt,previous_output_rad=a,output_rad=b,controller_delta_prev_rad=cur['delta_prev_rate_reference_rad'],rate_radps=delta/dt,previous_mode=prev['fallback_mode'],mode=cur['fallback_mode'],category=cat))
writecsv(OUT/'tables/output_rate_exceedances.csv',rate)

# Upper cycle overrun: timing scope and neighboring activity, no cause attribution.
timing=[json.loads(l) for l in gzip.open(OUT/'signals/s2_upper_stage_timing.jsonl.gz','rt')];worst=max(timing,key=lambda x:x['wall_ms']);dump(OUT/'tables/upper_deadline_overrun_record.json',worst)
fig,axs=plt.subplots(1,2,figsize=(12,5));tt=worst['t_start'];near=[z for z in timing if abs(z['t_start']-tt)<5];axs[0].plot([z['t_start']-tt for z in near],[z['wall_ms'] for z in near],'o-',label='Cycle body wall time');axs[0].plot([z['t_start']-tt for z in near],[z['cpu_ms'] for z in near],'s-',label='Thread CPU time');axs[0].axhline(500,color='#c22',ls='--',label='500 ms period');axs[0].set_ylabel('ms');axs[0].set_xlabel(f'Time from {clock(tt)} KST [s]');axs[0].legend();axs[0].grid(alpha=.2)
st=sorted(worst['stages'],key=lambda x:x['wall_ms'],reverse=True)[:6];y=np.arange(len(st));axs[1].barh(y-.18,[z['wall_ms'] for z in st],height=.35,label='wall');axs[1].barh(y+.18,[z['cpu_ms'] for z in st],height=.35,label='thread CPU');axs[1].set_yticks(y,[z['name'].replace('_',' ') for z in st]);axs[1].invert_yaxis();axs[1].set_xscale('log');axs[1].set_xlabel('ms (log scale)');axs[1].legend()
fig.suptitle(f'S2 A02: observed 500 ms cycle overrun / plan {worst["plan_seq"]}\n20261008_183952_scenario2_5524df — {clock(tt)} KST',fontsize=12);fig.tight_layout(rect=[0,.09,1,.9]);fig.text(.5,.025,'Body 760.540 ms; matched own-report overhead 0.272 ms. Wall/CPU difference does not establish the waiting or scheduling cause.',ha='center',fontsize=8)
for ext in ('pdf','png'):fig.savefig(OUT/f'figures/07_upper_deadline_overrun.{ext}',bbox_inches='tight')
plt.close(fig)

# A historical HIL input interruption, kept separate from the primary 30 attempts.
review=ROOT/'followup_20261008/scenario1_detailed_review';summary=json.loads((review/'summary.json').read_text());d=json.load(gzip.open(review/'telemetry.json.gz','rt'))['data'];a=summary['attempts'][4];gap=a['gaps']['/motive/vehicle/odom_map'];start=gap['from_t']-4;end=gap['to_t']+1;origin=gap['from_t'];runid=Path(summary['session']).name
old_sources=json.loads((review/'sources_sha256.json').read_text());assert all(sha(Path(p))==h for p,h in old_sources.items())
oldrows=[]
for p,h in old_sources.items():oldrows.append(dict(file=p,sha256=h,scope='historical input loss appendix',run_id=runid))
writecsv(OUT/'sources/historical_input_originals.csv',oldrows)
odom=[(t,v) for t,v in d['/motive/vehicle/odom_map'] if start<=t<=end];lo=np.array([v for t,v in d['/debug/lower_mpc_trace'] if start<=v[0]<=end]);rx={k:[t for t,v in d[k] if start<=t<=end] for k in ['/motive/vehicle/odom_map','/debug/lower_mpc_trace']}
writecsv(OUT/'signals/historical_s1_a05_odom.csv',[dict(receive_s=t,x_m=v[0],y_m=v[1],yaw_rad=v[2]) for t,v in odom]);writecsv(OUT/'signals/historical_s1_a05_lower.csv',[dict(stamp_ros_s=z[0],speed_mps=z[4],mode=int(z[12]),command_rad_equivalent=z[26]) for z in lo])
logs=[dict(receive_s=t,**v) for t,v in d['/rosout'] if start<=t<=end];writecsv(OUT/'signals/historical_s1_a05_rosout.csv',logs)
fig=plt.figure(figsize=(12,7));gs=fig.add_gridspec(3,2,width_ratios=[1,1.5]);ax=fig.add_subplot(gs[:,0]);r=np.array(summary['route']);ax.plot(r[:,0],r[:,1],'o--',c='#2754b8',label='SD path');tt=np.array([t for t,v in odom]);xy=np.array([v[:2] for t,v in odom]);xx=xy[:,0].copy();yy=xy[:,1].copy();splits=np.flatnonzero(np.diff(tt)>.5)+1;xx[splits]=np.nan;yy[splits]=np.nan;ax.plot(xx,yy,c='black',lw=2,label='Observed pose only');ax.scatter(*xy[tt<=origin][-1],c='#c22',s=60,label='Last valid pose');ax.scatter(*xy[-1],c='#198657',s=60,label='Pose returns');ax.set_xlim(float(xy[:,0].min())-7,float(xy[:,0].max())+7);ax.set_ylim(float(xy[:,1].min())-7,float(xy[:,1].max())+7);ax.set_aspect('equal');ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.legend(fontsize=8);ax.set_title('Unobserved motion is not interpolated')
for j,(col,label) in enumerate([(26,'Steering command [rad-equiv.]'),(12,'Lower mode'),(4,'Stored lower speed [m/s]')]):
    q=fig.add_subplot(gs[j,1]);q.plot(lo[:,0]-origin,lo[:,col],c='#225c99');q.axvspan(0,gap['seconds'],color='#efc760',alpha=.3);q.axvline(a['end']-origin,c='#c22',ls='--');q.set_ylabel(label);q.grid(alpha=.2);q.set_xlim(-4,gap['seconds']+1)
    if j==2:q.set_xlabel(f'Time from last pose at {clock(origin)} KST [s]');q.text(.03,.82,'Speed during the gap is stale state, not observed motion.',transform=q.transAxes,fontsize=8)
fig.suptitle(f'Historical input interruption: S1 A05 — {runid}\nPose gap {gap["seconds"]:.3f} s; pose_timeout at {clock(a["end"])} KST',fontsize=12);fig.tight_layout(rect=[0,.04,1,.93]);fig.text(.5,.015,'Historical cohort, excluded from latest-30 performance totals. Mode 4 also denotes batch permission stop, not only goal arrival.',ha='center',fontsize=8)
for ext in ('pdf','png'):fig.savefig(OUT/f'figures/08_historical_input_loss.{ext}',bbox_inches='tight')
plt.close(fig)
dump(OUT/'tables/historical_input_loss.json',dict(run_id=runid,attempt=5,scope='historical appendix, not primary30',gap=gap,termination_t_ros_s=a['end'],neutral_t_ros_s=a['terminal_neutral']['t'],source_review=str(review),cause='pose source became stale and heartbeat timeout logged; ultimate source of outage not isolated'))

# Machine-readable signal definitions and source/method mapping.
dictionary=[
 ('signals/s*_odometry.csv.gz','receive_s','s, Unix epoch','recorder host ROS/wall time','bag receipt','direct'),
 ('signals/s*_odometry.csv.gz','header_ros_s','s, Unix epoch','bridge/node clock; not sensor acquisition','message header','direct'),
 ('signals/s*_odometry.csv.gz','x_map_m,y_map_m,z_map_m','m','reanchored map; planar ENU convention','EKF output converted and reanchored; not independent truth','direct'),
 ('signals/s*_odometry.csv.gz','yaw_scalar_rad','rad','map +X, positive about +Z','orientation.z scalar; do not quaternion-decode','direct'),
 ('signals/s*_odometry.csv.gz','vx_map_mps,vy_map_mps','m/s','map axes (custom bridge convention)','finite differences with magnitude LPF alpha .4','direct'),
 ('signals/s*_lower_trace.csv.gz','stamp_ros_s','s, Unix epoch','node clock at trace publication','not control callback entry','direct'),
 ('signals/s*_lower_trace.csv.gz','x_map_m,y_map_m,yaw_scalar_rad,speed_mps','m,m,rad,m/s','latest cached map state','can be stale in neutral; not measured ground truth','direct'),
 ('signals/s*_lower_trace.csv.gz','lateral_error_m,body_heading_error_rad','m,rad','nearest dense lower-path point and tangent','retain only qp_valid OR error_feedback_available; neutral placeholder zeros excluded','direct'),
 ('signals/s*_lower_trace.csv.gz','output_command_rad_equivalent','rad-equivalent','ROS steering sign','virtual_steer_norm × .35; command not wheel measurement','direct'),
 ('signals/s*_lower_trace.csv.gz','mpc_first_command_rad,selected_command_rad,delta_prev_rate_reference_rad,last_applied_command_rad','rad command','controller sign','QP output / before filter / rate reference / command history; not measurements','direct'),
 ('signals/s*_lower_trace.csv.gz','estimated_effective_steering_rad','rad estimated','internal actuator model','not measured steering','direct'),
 ('signals/s*_lower_trace.csv.gz','applied_steer_norm,virtual_steer_norm,mavlink_steering_norm','dimensionless','respective output channel signs','MAVLink transmitted sign is opposite ROS; see source/HANDOFF','direct'),
 ('signals/s*_lower_trace.csv.gz','virtual_throttle,virtual_brake,mavlink_throttle_norm','dimensionless','command channel','virtual brake does not establish active PX4 braking','direct'),
 ('signals/s*_lower_trace.csv.gz','solver_only_ms','ms','steady_clock','solve_problem enter→return; NaN if uncalled','direct'),
 ('signals/s*_lower_trace.csv.gz','build_to_solver_return_ms','ms','steady_clock','solveTrackingMpc entry→solver return; excludes validation, command construction/publication and other callback work','direct'),
 ('signals/s*_lower_trace.csv.gz','nearest_dense_index,qp_valid,fallback_mode,error_feedback_available','index / flag / enum','none','modes 0 QP,1 previous sequence,2 hold,3 neutral invalid,4 goal OR supervisor stop,5 error feedback','direct'),
 ('signals/s*_lower_trace.csv.gz','objective,reference_curvature_per_m','scaled cost,1/m','lower reference','objective implementation units; curvature diagnostic only','direct'),
 ('signals/s*_lower_trace.csv.gz','actuator_model_enabled,actuator_delay_s,actuator_tau_s,point_to_rear_axle_m,command_scale_rad','flag,s,s,m,rad/unit','controller model','actual model parameters in trace','direct'),
 ('signals/s*_upper_events.jsonl.gz','t_start,t_emit,receive_s','s, Unix epoch','planner ROS clock / recorder clock','cycle entry / JSON creation / bag receipt','direct'),
 ('signals/s*_upper_events.jsonl.gz','preview_world,planned_world,published_world,retained_world,intervals_world','m','map','preview carries previous plan; published can truncate guarded tail; intervals are processed lateral slabs','direct'),
 ('signals/s*_upper_events.jsonl.gz','selected_corridors,candidate_counts,plan_seq,source_mode','index/count','per cycle','not persistent road or branch IDs; source_mode 1 nominal fallback','direct'),
 ('signals/s*_upper_events.jsonl.gz','solver_time_ms','ms','steady_clock','sum of actual solve_problem calls, enumeration and polishing included','direct'),
 ('signals/s*_upper_stage_timing.jsonl.gz','wall_ms,cpu_ms,stages,start_interval_ms,previous_report_ms','ms','steady_clock / CLOCK_THREAD_CPUTIME_ID','full planner body through local cleanup; own report cost arrives on next cycle','direct'),
 ('tables/s*_cycle_times.csv.gz','cycle_including_report_ms','ms','two matched consecutive cycle_seq','wall_ms[current]+previous_report_ms[next]; not callback queue delay; last sample may be missing','reconstructed'),
 ('signals/s*_grid_times.csv.gz','receive_s,header_s','s','recorder clock / source header','all observed grid stamps are zero; image age and source latency unavailable','direct'),
 ('scenes/*grid.npz','grid','occupancy integer','Vehicle x forward y left; res .3125m; origin(0,-20)','0 or1 drivable at threshold1; values>1 non-drivable; negative unknown','selected direct frame'),
 ('scenes/*road_mosaic.npz','road_votes,votes','count','post-hoc map .5m cells','grid every .75s, pixel stride2, pose interpolated between receive times; for visualization only','reconstructed'),
 ('signals/s*_lower_paths.jsonl.gz','receive_s,header_s,frame,xy,s','s,s,frame,m,m','map; path generation header','recorded atomic PathWithArcLength, no synthetic lower path','direct'),
 ('signals/s*_waypoints.csv','index,x_map_m,y_map_m','index,m,m','map','recorded global_path; latest cohorts bias disabled','direct'),
 ('signals/s*_B.csv.gz','B_before_m,B_m,threshold_m,candidate_m,candidate_step,candidate_interval','m,m,m,m,index,index','interval algorithm','diagnostics, not physical road width measurement','direct'),
 ('signals/s*_batch_status.jsonl.gz','phase_stamp_sec,phase,index,sequence','s/enum/index','runner ROS clock','running..coasting defines primary performance interval','direct'),
 ('tables/plan_interval_checks.csv.gz','lateral_slab_margin_m,violation_m','m','selected processed interval axis in map','projection test, 1mm tolerance, guarded tail excluded; not footprint clearance','reconstructed'),
 ('tables/trials.csv','observed_distance_running_m,observed_distance_through_coast_m','m','recorded EKF trajectory','sum consecutive 2D odom sample distances within phase; includes estimation noise','reconstructed'),
 ('tables/trials.csv','lateral_rmse_m,body_heading_rmse_rad','m,rad','lower path reference','unweighted sample RMSE over error-valid command modes during running; moving subset separately','reconstructed'),
 ('signals/*rosout*.csv*','receive_s,name,level,msg','s/string/enum','recorder clock','recorded node messages; may be throttled; do not count warnings as exact event counts','direct')]
writecsv(OUT/'sources/signal_dictionary.csv',[dict(file=f,fields=k,units=u,frame_and_time=c,meaning=m,origin=o) for f,k,u,c,m,o in dictionary])
print('Supplement complete',flush=True)
