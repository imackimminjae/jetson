"""Scenario3 multi-run analysis (bag + tlog). Segments by route progress: A 0-60 (start + first bend),
B 60-115 (first junction / SE road), C 115-145 (approach + last corner), END >145 (final NE leg, reported separately)."""
import pickle, numpy as np, time, json, sys, glob, re, os
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import q2yaw, eff_steer, project, wrap, lpf
from scipy.signal import butter, filtfilt
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def clock(t): return time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)
SEG=(('A_0_60',0,60),('B_60_115',60,115),('C_115_145',115,145),('END_145+',145,1e9))
RUNS=[
 ('S1_0141_rd120','/home/imac/ros2_ws/drive_debug_20261006_014143_light','/home/imac/ros2_ws/followup_20261006/sc3_0141_0143/tlog_sc3.pkl',(229.413925,-304.748047,-1.049644),-0.521153,'upper_planner_node_27960_1791218499871.log'),
 ('S2_0143_rd120','/home/imac/ros2_ws/drive_debug_20261006_014303_light','/home/imac/ros2_ws/followup_20261006/sc3_0141_0143/tlog_sc3.pkl',(360.707794,-270.353394,0.838671),-2.409467,'upper_planner_node_28944_1791218578855.log'),
 ('R1_032358_rd120','/home/imac/ros2_ws/drive_debug_20261006_032358_light','tlog_0320.pkl',(347.580505,-216.533920,1.385218),-2.956015,'upper_planner_node_45514_1791224635438.log'),
 ('R2_032453_rd120','/home/imac/ros2_ws/drive_debug_20261006_032453_light','tlog_0320.pkl',(239.940048,-88.028641,2.628966),2.083423,'upper_planner_node_46569_1791224690380.log'),
 ('R3_032545_rd120','/home/imac/ros2_ws/drive_debug_20261006_032545_light','tlog_0320.pkl',(175.139343,-186.324524,-1.088879),-0.481917,'upper_planner_node_47773_1791224742803.log'),
 ('R4_032651_rd120','/home/imac/ros2_ws/drive_debug_20261006_032651_light','tlog_0320.pkl',(281.981201,-148.862091,1.475190),-3.045987,'upper_planner_node_49020_1791224808611.log'),
 ('R5_032741_rd150','/home/imac/ros2_ws/drive_debug_20261006_032741_light','tlog_0320.pkl',(147.224991,-23.777575,2.729237),1.983152,'upper_planner_node_49932_1791224858419.log'),
 ('R6_032829_rd150','/home/imac/ros2_ws/drive_debug_20261006_032829_light','tlog_0320.pkl',(62.946770,-98.653969,-1.153243),-0.417553,'upper_planner_node_50766_1791224905756.log')]
cache={}
def tlog(p):
    if p not in cache:
        d=pickle.load(open(p,'rb')); cache[p]=d['data'] if 'data' in d else d
    return cache[p]
bb,ab=butter(2,0.3/(10/2),'high')
out={}; fig,axs=plt.subplots(2,1,figsize=(16,10),sharex=True); fig2,ax2=plt.subplots(figsize=(10,9))
ax2.plot(GP[:,0],GP[:,1],'k--o',label='scenario3 route')
for ri,(name,bag,tl,src,yd,ulog) in enumerate(RUNS):
    d=pickle.load(open(bag+'/analysis/decoded.pkl','rb')); T=tlog(tl)
    tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); ttr=tr[:,0]
    a,b=ttr[0]-2,ttr[-1]+15
    sel=lambda typ:[x for x in T[typ] if a<=x['_t']<=b]
    lp=np.array([(x['_t'],x['x'],x['y']) for x in sel('LOCAL_POSITION_NED')])
    hs=np.array([(x['_t'],x['yawspeed'],np.hypot(x['vx'],x['vy'])/100) for x in sel('HIL_STATE_QUATERNION')])
    cl=np.array([(x['_t'],x['param1'],x['param2']*0.35) for x in sel('COMMAND_LONG') if x['command']==187])
    sx,sy,_=src; c_,s_=np.cos(yd),np.sin(yd); xe,ye=lp[:,2],lp[:,1]
    mx=187+c_*(xe-sx)-s_*(ye-sy); my=220+s_*(xe-sx)+c_*(ye-sy); prog,lat=project(GP,mx,my)
    # sanity vs bag odom
    od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od])
    alignerr=np.median(np.hypot(np.interp(ot,lp[:,0],mx)-oxy[:,0],np.interp(ot,lp[:,0],my)-oxy[:,1]))
    t_on=cl[cl[:,1]>0][0,0]
    L=open(os.path.expanduser('~/.ros/log/'+ulog)).read()
    goals=[(float(m.group(1)),float(m.group(2))) for m in re.finditer(r'\[(\d+\.\d+)\] \[upper_planner_node\]: goal reached: remaining=[0-9.]+ direct=([0-9.]+)',L)]
    fails_log=[float(m.group(1)) for m in re.finditer(r'\[(\d+\.\d+)\] \[upper_planner_node\]: egocentric MIQP failed',L)]
    th=hs[:,0]; v=hs[:,2]; yr=hs[:,1]; deff=eff_steer(yr,v); uu=np.interp(th,cl[:,0],cl[:,2])
    ph=np.interp(th,lp[:,0],prog); pc=np.interp(cl[:,0],lp[:,0],prog); pt=np.interp(ttr,lp[:,0],prog)
    t_goal=goals[0][0] if goals else b
    # steering high-pass on 10 Hz resampled command
    tg=np.arange(t_on,min(t_goal,cl[-1,0]),0.1); sg=np.interp(tg,cl[:,0],cl[:,2]); hp=filtfilt(bb,ab,sg) if len(sg)>20 else sg*0; pg=np.interp(tg,lp[:,0],prog)
    r={'align_check_median_m':round(float(alignerr),3),'goal':(clock(goals[0][0]),goals[0][1]) if goals else None,'progress_max':round(float(prog.max()),1)}
    # junction choices
    def passed(cond): return bool(np.any(cond))
    r['J1_(200,160)']='SE main (correct)' if passed((mx>212)&(my>150)&(my<162)) and not passed((mx<212)&(mx>195)&(my<148)) else ('SW branch (WRONG)' if passed((mx<212)&(mx>195)&(my<148)) else 'n/a')
    r['J2_(255,140)']='NE (correct)' if passed((mx>262)&(my>150)) else ('E straight (WRONG)' if passed((mx>270)&(my<140)) else 'not reached')
    ev=d['/debug/upper_branch_event']
    r['upper_fails']=[(clock(x['t']),round(float(np.interp(x['t'],lp[:,0],prog)),0),(round(x['data']['x']),round(x['data']['y']))) for x in ev if not x['data']['valid_plan']]
    r['upper_fails_log_total']=len(fails_log); r['upper_fails_log_progress']=[round(float(np.interp(t,lp[:,0],prog)),0) for t in fails_log]
    g=np.diff(cl[:,0]); r['cmd_gaps>0.3s']=[(clock(cl[i,0]),round(x,2),round(float(pc[i]))) for i,x in enumerate(g) if x>0.3 and t_on<=cl[i,0]<=t_goal]
    trg=np.diff(ttr); r['lower_trace_gaps>0.3s']=[(clock(ttr[i]),round(x,2)) for i,x in enumerate(trg) if x>0.3]
    neu=[(clock(cl[i,0]),round(float(pc[i]))) for i in range(1,len(cl)) if cl[i,1]==0 and cl[i,2]==0 and cl[i-1,1]>0 and t_on<cl[i,0]<t_goal]; r['neutral_starts(time,prog)']=neu
    for sn,p0,p1 in SEG:
        mh=(ph>=p0)&(ph<p1)&(th>=t_on)&(th<=t_goal)&(v>1); mc=(pc>=p0)&(pc<p1)&(cl[:,0]>=t_on)&(cl[:,0]<=t_goal); mt=(pt>=p0)&(pt<p1)&(tr[:,10]==1); mg=(pg>=p0)&(pg<p1)
        if mh.sum()<10 or mc.sum()<10: continue
        sc=cl[mc,2]; u=sc[np.r_[True,np.abs(np.diff(sc))>1e-6]]; dur=max(1e-3,cl[mc,0][-1]-cl[mc,0][0])
        f=json.load(open(bag+'/analysis/features.json')); fp=np.interp([x['t'] for x in f],lp[:,0],prog)
        jumps=[abs(x.get('h1_jump_deg',0)) for x,q in zip(f,fp) if p0<=q<p1 and x['valid']]
        # k1 corridor flips: consecutive valid plans where k=1 had >=2 candidates and selected index changed
        k1=[(x['data']['selected_corridors'][1] if x['data']['valid_plan'] and len(x['data']['selected_corridors'])>1 else None, (x['data'].get('candidate_counts') or [0,0])[1]) for x in ev if p0<=np.interp(x['t'],lp[:,0],prog)<p1]
        flips=sum(1 for (s0,c0),(s1,c1) in zip(k1[:-1],k1[1:]) if s0 is not None and s1 is not None and c1>=2 and s0!=s1)
        r[sn]=dict(cmd_p95=round(float(np.degrees(np.quantile(np.abs(sc),.95))),1),sat=round(float(np.mean(np.abs(sc)>=0.298)),3),
            rev_per_s=round(float(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)/dur),2),hp_rms=round(float(np.degrees(np.sqrt(np.mean(hp[mg]**2)))) if mg.sum() else np.nan,2),
            yawrate_p95=round(float(np.degrees(np.quantile(np.abs(yr[mh]),.95))),1),opp=round(float(np.mean((np.sign(uu[mh])*np.sign(deff[mh])<0)&(np.abs(uu[mh])>np.radians(3)))),3),
            lat_p95=round(float(np.quantile(np.abs(tr[mt,6]),.95)),2) if mt.sum() else None,head_p95=round(float(np.degrees(np.quantile(np.abs(tr[mt,7]),.95))),1) if mt.sum() else None,
            route_dev_p95=round(float(np.quantile(np.abs(np.interp(th[mh],lp[:,0],lat)),.95)),2),plan_jump_p50_max=[round(float(np.median(jumps)),1),round(float(max(jumps)),1)] if jumps else None,k1_flips=flips,
            tau_rmse074=round(float(np.degrees(np.sqrt(np.mean((lpf(th,uu,0.74)[mh]-deff[mh])**2)))),2),v_p50=round(float(np.median(v[mh])),2))
    out[name]=r
    col='C%d'%ri; mm=(th>=t_on)&(th<=t_goal)
    axs[0].plot(pc[(cl[:,0]>=t_on)&(cl[:,0]<=t_goal)],np.degrees(cl[(cl[:,0]>=t_on)&(cl[:,0]<=t_goal),2]),color=col,lw=.9,label=name)
    axs[1].plot(ph[mm],np.degrees(yr[mm]),color=col,lw=.9,label=name)
    ax2.plot(mx[(lp[:,0]>=t_on)],my[(lp[:,0]>=t_on)],color=col,lw=1,label=name)
for a in axs:
    for x in (60,115,145): a.axvline(x,color='k',ls=':')
    a.grid(True); a.legend(fontsize=7,ncol=4)
axs[0].set_ylabel('steer cmd deg (MAVLink sign)'); axs[1].set_ylabel('SIH yaw rate deg/s'); axs[1].set_xlabel('progress along scenario3 route (m); dotted: 60 / 115 / 145(end leg)')
fig.tight_layout(); fig.savefig('sc3_runs_steering.png',dpi=80); ax2.set_aspect('equal'); ax2.grid(alpha=.3); ax2.legend(fontsize=7); fig2.tight_layout(); fig2.savefig('sc3_runs_tracks.png',dpi=80)
json.dump(out,open('sc3_runs_summary.json','w'),indent=1,default=str)
for k,r in out.items():
    print('=====',k,{kk:vv for kk,vv in r.items() if not isinstance(vv,dict)})
    for sn,_,_ in SEG:
        if sn in r: print('   ',sn,r[sn])
