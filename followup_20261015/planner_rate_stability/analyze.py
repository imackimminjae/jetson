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
    pairs=mask[:-1]&mask[1:]&(dt>.03)&(dt<.25)
    changes=np.diff(steer)[pairs];rates=changes/dt[pairs]
    distance=np.sum(.5*(l[:-1,4]+l[1:,4])[pairs]*dt[pairs])
    active_time=float(np.sum(dt[pairs]))
    # 0.5-second smoothing separates rapid command changes from slower turns.
    rt=np.arange(t[0],t[-1],.1);command=np.interp(rt,t,steer)
    good=np.interp(rt,t,mask.astype(float))>.99
    jitter=command-uniform_filter1d(command,5,mode='nearest')
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

groups=[];sources={};validation=[];curves={}
for p in OUT.glob('decoded*.pkl'):
    d=pickle.load(p.open('rb'));cfg=yaml.safe_load(d['config']);rate=cfg['upper_planner_node']['ros__parameters']['upper_planner_rate_hz']
    groups.append((rate,d,cfg))
groups.sort(key=lambda v:v[0])
reports=[]
for rate,d,cfg in groups:
    D=d['data'];b=ROOT/'drive_batches'/d['batch'];route=np.asarray(d['route']);phase=[v for t,v in D['/knu_batch/status']]
    meta=yaml.safe_load((b/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information'];counts={v['topic_metadata']['name']:v['message_count'] for v in meta['topics_with_message_count']}
    for topic,vals in D.items():assert len(vals)==counts[topic],(topic,len(vals),counts[topic])
    for path in b.rglob('*'):
        if path.is_file():sources[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    attempts=[];curves[rate]=[]
    for i,row in enumerate(d['summary'],1):
        start=next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='running')
        end=next(v['phase_stamp_sec'] for v in phase if v['index']==i and v['phase']=='coasting')
        l=np.array([v for t,v in D['/debug/lower_mpc_trace'] if start<=t<end])
        od=np.array([v for v in D['/motive/vehicle/odom_map'] if start<=v[0]<end])
        ev=[v for t,v in D['/debug/upper_branch_event'] if start<=v['t_start']<end]
        cy=[v for t,v in D['/debug/upper_stage_timing'] if start<=v['t_start']<end]
        grid=np.array([t for t,v in D['/bev/occupancy_grid'] if start<=t<end])
        assert len(l)>100 and row['outcome']=='completed'
        speed=np.hypot(od[:,5],od[:,6]);s,offset,cum=project(od[:,1:3],route)
        sl,el,_=project(l[:,1:3],route)
        moving=l[:,4]>1
        valid=(l[:,10]>0)|(l[:,27]>0)
        command=steering_metrics(l,moving,cfg['lower_tracking_mpc_node']['ros__parameters']['lower_max_steering_angle_rad'])
        # Use common spatial ranges for segmented comparisons, independent of time.
        segment_metrics={}
        for name,lo,hi in [('approach',10,90),('turn',90,155),('exit',155,175)]:
            mask=moving&(sl>=lo)&(sl<hi)
            segment_metrics[name]=steering_metrics(l,mask,.3)
            mv=mask&valid
            segment_metrics[name].update(lateral_rmse_m=rms(l[mv,6]),heading_rmse_deg=rms(np.degrees(l[mv,7])))
        mask=moving&valid
        # Estimate physical yaw-rate from measured pose on a uniform 20 Hz grid.
        t=np.arange(od[0,0],od[-1,0],.05);yaw=np.unwrap(od[:,3]);yr=savgol_filter(np.interp(t,od[:,0],yaw),9,2,deriv=1,delta=.05,mode='interp');v=np.interp(t,od[:,0],speed)
        gyro=np.degrees(yr);ymask=v>1
        yawjerk=np.diff(gyro)/.05;yp=ymask[1:]&ymask[:-1]
        plan_jumps=[];prev=None
        for e in ev:
            if not e['valid_plan']:continue
            path=np.array(e['published_world'])
            if len(path)>1:
                if prev is not None:
                    previous_path=prev[1];di=np.linalg.norm(previous_path-np.array([e['x'],e['y']]),axis=1);k=int(np.argmin(di));k=min(k,len(previous_path)-2)
                    old=np.arctan2(*(previous_path[k+1]-previous_path[k])[::-1]);new=np.arctan2(*(path[1]-path[0])[::-1]);plan_jumps.append(abs(np.degrees(wrap(new-old))))
                prev=(e['t_start'],path)
        a=dict(attempt=i,outcome=row['outcome'],start=start,end=end,duration_sec=float(row['duration_sec']),moving_mean_speed_mps=float(np.mean(l[moving,4])),initial_pose=od[0,1:4].tolist(),steering=command,segments=segment_metrics,lateral_rmse_m=rms(l[mask,6]),lateral_abs_p95_m=p95(abs(l[mask,6])),lateral_abs_max_m=float(np.max(abs(l[mask,6]))),heading_rmse_deg=rms(np.degrees(l[mask,7])),heading_abs_p95_deg=p95(abs(np.degrees(l[mask,7]))),waypoint_polyline_offset_rms_m=rms(offset[speed>1]),waypoint_polyline_offset_p95_m=p95(abs(offset[speed>1])),yaw_rate_abs_p95_degps=p95(abs(gyro[ymask])),yaw_accel_rms_degps2=rms(yawjerk[yp]),yaw_accel_abs_p95_degps2=p95(abs(yawjerk[yp])),upper_attempts=len(ev),upper_failures=sum(not e['valid_plan'] for e in ev),upper_failure_reasons=dict(Counter(e['reason'] for e in ev if not e['valid_plan'])),lower_qp_invalid=int(np.sum(moving&(l[:,10]==0))),lower_without_qp_or_fallback=int(np.sum(moving&(~valid))),fallback_modes=dict(Counter(int(x) for x in l[moving,12])),upper_wall_ms=stat([c['wall_ms'] for c in cy]),upper_over_period=sum(c['wall_ms']>1000/rate for c in cy),upper_interval_ms=stat([c['start_interval_ms'] for c in cy if c['start_interval_ms'] is not None]),grid_gap_max_s=float(np.diff(grid).max()),grid_gaps_over_0_5s=int(np.sum(np.diff(grid)>.5)),lower_gap_max_s=float(np.diff(l[:,0]).max()),plan_first_heading_jump_deg=stat(plan_jumps))
        attempts.append(a);curves[rate].append(dict(s=s,offset=offset,odom=od,lower=l,lower_s=sl,events=ev,start=start,end=end))
        print(rate,i,'TV/s',round(command['variation_deg_per_sec'],2),'TV/100m',round(command['variation_deg_per_100m'],1),'steerRate95',round(command['rate_p95_degps'],1),'jitter',round(command['rapid_residual_rms_deg'],2),'sat',round(command['limit_ratio_pct'],1),'e_y',round(a['lateral_rmse_m'],3),'e_heading',round(a['heading_rmse_deg'],1),'bevMaxGap',round(a['grid_gap_max_s'],3),'upfails',a['upper_failures'],flush=True)
    # Spatial alignment avoids averaging turns that occur at different times.
    smin=max(c['s'].min() for c in curves[rate]);smax=min(c['s'].max() for c in curves[rate]);sg=np.arange(max(5,smin),min(180,smax),.5)
    xy=[];st=[];lat=[];angle=[]
    for c in curves[rate]:
        ss=np.maximum.accumulate(c['s']);uniq,idx=np.unique(ss,return_index=True)
        xy.append(np.column_stack([np.interp(sg,uniq,c['odom'][idx,k]) for k in [1,2]]))
        ls=np.maximum.accumulate(c['lower_s']);u,j=np.unique(ls,return_index=True)
        st.append(np.interp(sg,u,np.degrees(-c['lower'][j,17]*c['lower'][j,31])))
        lat.append(np.interp(sg,u,c['lower'][j,6]));angle.append(np.interp(sg,u,np.degrees(c['lower'][j,7])))
    xy=np.asarray(xy);spread=np.sqrt(np.var(xy[:,:,0],axis=0)+np.var(xy[:,:,1],axis=0));cm=np.mean(xy,axis=0)
    spatial=dict(common_progress_min_m=float(sg[0]),common_progress_max_m=float(sg[-1]),rms_spread_m=rms(spread),p95_spread_m=p95(spread),max_spread_m=float(spread.max()),turn_rms_spread_m=rms(spread[(sg>=90)&(sg<155)]),exit_rms_spread_m=rms(spread[sg>=155]))
    np.savez_compressed(OUT/f'spatial_{rate:g}hz.npz',s=sg,xy=xy,mean_xy=cm,spread=spread,steering=np.array(st),lateral=np.array(lat),heading=np.array(angle))
    reports.append(dict(rate_hz=rate,batch=d['batch'],attempts=attempts,spatial=spatial))
    validation.append(dict(batch=d['batch'],selected_topic_counts_match_metadata=True,attempts=len(attempts)))

(OUT/'metrics.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n')
(OUT/'sources_sha256.json').write_text(json.dumps(sources,indent=2)+'\n')
(OUT/'validation.json').write_text(json.dumps({'batches':validation,'source_hashes_saved':True,'operating_code_changed':False},indent=2)+'\n')
rows=[]
for r in reports:
    for a in r['attempts']:rows.append(dict(rate_hz=r['rate_hz'],attempt=a['attempt'],duration_sec=a['duration_sec'],speed_mps=a['moving_mean_speed_mps'],**a['steering'],lateral_rmse_m=a['lateral_rmse_m'],heading_rmse_deg=a['heading_rmse_deg'],yaw_accel_rms_degps2=a['yaw_accel_rms_degps2'],upper_failures=a['upper_failures'],lower_qp_invalid=a['lower_qp_invalid'],grid_gap_max_s=a['grid_gap_max_s']))
with (OUT/'per_attempt.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)

COLORS=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
fig,axs=plt.subplots(2,4,figsize=(16,9))
for j,r in enumerate(reports):
    rate=r['rate_hz'];z=np.load(OUT/f'spatial_{rate:g}hz.npz');route=groups[j][1]['route'];route=np.asarray(route)
    for ax in axs[:,j]:ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3)
    for i,c in enumerate(curves[rate]):
        for ax in axs[:,j]:ax.plot(c['odom'][:,1],c['odom'][:,2],color=COLORS[i],lw=1.4,label=f'{i+1}회')
    axs[0,j].set_title(f'{rate:g} Hz · 목표 접근 5/5\n회차 궤적 편차 RMS {r["spatial"]["rms_spread_m"]:.2f} m')
    axs[0,j].set_xlim(95,180);axs[0,j].set_ylim(-145,10);axs[0,j].legend(fontsize=8)
    axs[1,j].set_xlim(102,153);axs[1,j].set_ylim(-35,8);axs[1,j].set_title('회전 구간 확대')
    for ax in axs[:,j]:ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
fig.suptitle('BEV 수신 회복 후 · 시나리오1 · 플래너 주파수별 5회 궤적',fontsize=16)
fig.tight_layout();fig.savefig(OUT/'trajectories.png',dpi=170)

fig,axs=plt.subplots(4,4,figsize=(16,11),sharex='col')
for j,r in enumerate(reports):
    rate=r['rate_hz'];z=np.load(OUT/f'spatial_{rate:g}hz.npz');s=z['s']
    for i in range(5):
        axs[0,j].plot(s,z['steering'][i],lw=1,color=COLORS[i],alpha=.8)
        axs[1,j].plot(s,z['lateral'][i],lw=1,color=COLORS[i],alpha=.8)
        axs[2,j].plot(s,z['heading'][i],lw=1,color=COLORS[i],alpha=.8)
    axs[3,j].plot(s,z['spread'],color='#333333')
    axs[0,j].set_title(f'{rate:g} Hz')
    for k,lim in enumerate([(-19,19),(-2.2,2.2),(-45,45),(0,2)]):
        axs[k,j].set_ylim(*lim);axs[k,j].grid(alpha=.2);axs[k,j].axvspan(90,155,color='#aaaaaa',alpha=.1)
    axs[3,j].set_xlabel('웨이포인트 경로 진행거리 [m]')
for ax,label in zip(axs[:,0],['송신 조향 명령 [도 환산]','MPC 횡오차 [m]','MPC 방향 오차 [도]','회차 궤적 편차 [m]']):ax.set_ylabel(label)
fig.suptitle('같은 진행 위치에서 비교 · 회색 영역: 회전 구간 (90–155 m)',fontsize=15)
fig.tight_layout();fig.savefig(OUT/'spatial_signals.png',dpi=160)

summary=[]
for r in reports:
    a=r['attempts'];entry={'rate_hz':r['rate_hz'],'repeatability':r['spatial']}
    for name in ['variation_deg_per_sec','variation_deg_per_100m','rate_p95_degps','rapid_residual_rms_deg','limit_ratio_pct']:
        entry[name]=stat([x['steering'][name] for x in a])
    for name in ['duration_sec','moving_mean_speed_mps','lateral_rmse_m','heading_rmse_deg','yaw_accel_rms_degps2','grid_gap_max_s']:
        entry[name]=stat([x[name] for x in a])
    entry['upper_failures_total']=sum(x['upper_failures'] for x in a);entry['upper_attempts_total']=sum(x['upper_attempts'] for x in a);entry['lower_qp_invalid_total']=sum(x['lower_qp_invalid'] for x in a)
    summary.append(entry)
(OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print('SUMMARY',json.dumps(summary,ensure_ascii=False))
