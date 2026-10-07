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
GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)
TAGS=['032741','032829','043316','043401','044438','044521','044738']
SEG=[('A',0,60),('B',60,115),('C',115,145)]
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
    tlp=ROOT/'followup_20261006'/('sc3_0323_0328/tlog_0320.pkl' if tag.startswith('03') else 'runs_0427_0448/tlog.pkl')
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
    s={'bag':str(folder),'upper_log':str(ul),'first_throttle':clock(on),'bag_start':clock(ot[0]),'bag_start_lag_s':float(ot[0]-on),'r_dpsi':cfg['upper_planner_node']['ros__parameters']['upper_r_dpsi'], 'rd':cfg['lower_tracking_mpc_node']['ros__parameters']['lower_rd_steering_rate'],'goal_direct_m':goals[0][1] if goals else None, 'upper_failures_all':[clock(t) for t in fails], 'upper_failures_in_bag_progress':[float(np.interp(t,ot,prog)) for t in fails if ot[0]<=t<=ot[-1]], 'max_command_gap_s':float(max(np.diff(cl[(cl[:,0]>=on)&(cl[:,0]<=off),0]))), 'max_lower_trace_gap_s':float(max(np.diff(tr[:,0]))), 'lower_modes':{str(int(k)):int(np.sum(tr[:,12]==k)) for k in np.unique(tr[:,12])},'segments':{}}
    for sn,a,b in SEG:
        m=(ph>=a)&(ph<b)&(vv>1); mt=(pt>=a)&(pt<b)&(tr[:,10]==1)&(tr[:,0]>=aa)&(tr[:,0]<=zz)
        er=[e for e in evrows if a<=e['progress']<b and e['valid']]
        jumps=[abs(e['previous_plan_lateral_m'][0]) for e in er if e['previous_plan_lateral_m'] and e['previous_plan_lateral_m'][0] is not None]
        if not np.any(m):continue
        s['segments'][sn]={'duration_s':float(m.sum()*.1),'sat_fraction':float(np.mean(abs(cmd[m])>=.298)), 'steering_highpass_rms_deg':float(np.sqrt(np.mean(hp[m]**2))),'yawrate_abs_p95_deg_s':q95(yaw[m]),'speed_median_m_s':float(np.median(vv[m])),'local_path_lateral_p95_m':q95(tr[mt,6]),'local_path_heading_p95_deg':q95(np.degrees(tr[mt,7])), 'k1_previous_plan_shift_abs_p95_m':q95(jumps),'shift_sample_count':len(jumps)}
    s['J1']='SE' if np.any((oxy[:,0]>212)&(oxy[:,1]>150)&(oxy[:,1]<162)) else 'unconfirmed'
    s['J2']='NE' if np.any((oxy[:,0]>262)&(oxy[:,1]>150)) else ('E wrong' if np.any((oxy[:,0]>270)&(oxy[:,1]<140)) else 'unconfirmed')
    summaries[tag]=s; details[tag]=evrows; series[tag]={'oxy':oxy,'prog':prog,'t':grid,'p':ph,'cmd':np.degrees(cmd),'yawrate':yaw,'events':events,'lower':tr,'lower_progress':pt}
    print(tag,json.dumps(s,ensure_ascii=False))
(HERE/'summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
(HERE/'plan_events.json').write_text(json.dumps(details,ensure_ascii=False,indent=2)+'\n')
(HERE/'inputs_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
pickle.dump(series,(HERE/'series.pkl').open('wb'))
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
colors={'032741':'#999999','032829':'#bbbbbb','043316':'#e09a35','043401':'#45a69c','044438':'#da5555','044521':'#1c76ac','044738':'#aa59b5'}
for tag in TAGS:
    d=series[tag];s=summaries[tag];col=colors[tag]; label=tag[:2]+':'+tag[2:4]+':'+tag[4:]+f" r={s['r_dpsi']:g} {s['J2']}"
    axes[0,0].plot(d['oxy'][:,0],d['oxy'][:,1],c=col,label=label,lw=1.5)
    if tag.startswith('04'):
        axes[0,1].plot(d['p'],d['cmd'],c=col,lw=1,label=label)
        es=[e for e in details[tag] if e['previous_plan_lateral_m'] and e['previous_plan_lateral_m'][0] is not None]
        axes[1,0].plot([e['progress'] for e in es],[e['previous_plan_lateral_m'][0] for e in es],'.-',c=col,lw=1)
        tr=d['lower']; m=tr[:,10]==1;axes[1,1].plot(d['lower_progress'][m],np.degrees(tr[m,7]),c=col,lw=1)
axes[0,0].plot(*GP.T,'k--',lw=1,label='지정 경로');axes[0,0].set_aspect('equal');axes[0,0].set_xlim(180,315);axes[0,0].set_ylim(120,225);axes[0,0].legend(fontsize=8)
axes[0,0].set_title('기록된 차량 궤적 (회색: 이전 동일 설정 성공 2회)');axes[0,0].set_xlabel('지도 X (m)');axes[0,0].set_ylabel('지도 Y (m)')
axes[0,1].set_title('ROS 부호로 환산한 실제 전송 조향 명령');axes[0,1].set_ylabel('조향 (°)');axes[0,1].axhline(np.degrees(.3),c='k',ls=':',lw=.7);axes[0,1].axhline(-np.degrees(.3),c='k',ls=':',lw=.7)
axes[1,0].set_title('근거리 k=1 새 계획의 직전 계획 대비 횡이동');axes[1,0].set_ylabel('횡이동 (m)')
axes[1,1].set_title('하위 제어기의 지역 경로 대비 헤딩 오차');axes[1,1].set_ylabel('헤딩 오차 (°)')
for ax in [axes[0,1],axes[1,0],axes[1,1]]:
    ax.set_xlim(30,145);ax.axvline(60,c='#888',ls=':');ax.axvline(115,c='#888',ls=':');ax.set_xlabel('지정 경로에 투영한 진행 거리 (m)');ax.grid(alpha=.2)
fig.suptitle('최근 scenario3 기록 비교: 분기 선택과 조향 안정성\n종점 구간 제외 · bag 시작 전 자료는 궤적/계획 그래프에 없음',fontsize=15)
fig.savefig(HERE/'recent_comparison.png',dpi=170);plt.close(fig)

# Final-junction plan snapshots: same settings, one correct and two incorrect runs.
fig,axes=plt.subplots(1,3,figsize=(16,6),layout='constrained')
for ax,tag in zip(axes,['044438','044521','044738']):
    d=series[tag];ax.plot(*GP.T,'k--',alpha=.5,label='지정 경로');ax.plot(d['oxy'][:,0],d['oxy'][:,1],c='#888',label='실제 궤적')
    es=[e for e in d['events'] if e['valid_plan'] and 100<float(project(GP,np.array([e['x']]),np.array([e['y']]))[0][0])<135]
    for i,e in enumerate(es):
        pl=np.array(e['planned_world']);ax.plot(*pl.T,'o-',ms=2,c=plt.cm.viridis(i/max(1,len(es)-1)),label=clock(e['t_start'])[3:8])
    ax.set_xlim(230,292);ax.set_ylim(122,176);ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8,ncol=2);ax.set_title(tag+' · '+summaries[tag]['J2']);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)')
fig.suptitle('마지막 갈림길의 연속 계획 · 세 주행 모두 r_dpsi=3, rd=150, 전환 10/15\n색 선은 매 주기 새로 계산한 계획; 지도 원단면/비용의 원인은 별도 검증 필요',fontsize=14)
fig.savefig(HERE/'final_junction_plans.png',dpi=170);plt.close(fig)
