"""Generic multi-run analysis for scenario1 / scenario3 (bag + tlog). End leg excluded from steering stats."""
import pickle, numpy as np, time, json, sys, re, os, collections
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import eff_steer, project, lpf
from scipy.signal import butter, filtfilt
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def clock(t): return time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
R={'scenario1':dict(GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float),target=(150,-140),
     seg=(('straight_0_88',0,88),('J1_Scurve_88_125',88,125),('J2_turn_125_150',125,150)),end=150),
   'scenario3':dict(GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float),target=(187,220),
     seg=(('A_0_60',0,60),('B_60_115',60,115),('C_115_145',115,145)),end=145)}
def junctions(route,mx,my):
    if route=='scenario1':
        j1=next((('right' if x>103 else 'LEFT(wrong)') for x,y in zip(mx,my) if y>-38),'n/a')
        j2='EAST(correct)' if np.any((mx>140)&(my<8)&(my>-12)) else ('NORTH(wrong)' if np.any((my>15)&(mx<135)) else 'n/a')
        return j1,j2
    j1='SE(correct)' if np.any((mx>212)&(my>150)&(my<162)) and not np.any((mx<212)&(mx>195)&(my<148)) else ('SW(wrong)' if np.any((mx<212)&(mx>195)&(my<148)) else 'n/a')
    j2='NE(correct)' if np.any((mx>262)&(my>150)) else ('E(wrong)' if np.any((mx>270)&(my<140)) else 'not reached')
    return j1,j2
bb,ab=butter(2,0.3/5,'high')
def analyze(name,route,bag,tl,src,yd,ulog,llog,cfg):
    G=R[route]; GP=G['GP']; d=pickle.load(open(bag+'/analysis/decoded.pkl','rb')); T=pickle.load(open(tl,'rb')); T=T['data'] if 'data' in T else T
    tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); ttr=tr[:,0]; a,b=ttr[0]-2,ttr[-1]+15
    sel=lambda typ:[x for x in T[typ] if a<=x['_t']<=b]
    lp=np.array([(x['_t'],x['x'],x['y']) for x in sel('LOCAL_POSITION_NED')]); hs=np.array([(x['_t'],x['yawspeed'],np.hypot(x['vx'],x['vy'])/100) for x in sel('HIL_STATE_QUATERNION')])
    cl=np.array([(x['_t'],x['param1'],x['param2']*0.35) for x in sel('COMMAND_LONG') if x['command']==187])
    sx,sy,_=src; c_,s_=np.cos(yd),np.sin(yd); xe,ye=lp[:,2],lp[:,1]; tx,ty=G['target']
    mx=tx+c_*(xe-sx)-s_*(ye-sy); my=ty+s_*(xe-sx)+c_*(ye-sy); prog,lat=project(GP,mx,my)
    od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od])
    aerr=np.median(np.hypot(np.interp(ot,lp[:,0],mx)-oxy[:,0],np.interp(ot,lp[:,0],my)-oxy[:,1]))
    t_on=cl[cl[:,1]>0][0,0]; U=open(os.path.expanduser('~/.ros/log/'+ulog)).read(); Lw=open(os.path.expanduser('~/.ros/log/'+llog)).read()
    goals=[(float(m.group(1)),float(m.group(2))) for m in re.finditer(r'\[(\d+\.\d+)\] \[upper_planner_node\]: goal reached: remaining=[0-9.]+ direct=([0-9.]+)',U)]
    fl=[float(m.group(1)) for m in re.finditer(r'\[(\d+\.\d+)\] \[upper_planner_node\]: egocentric MIQP failed',U)]
    t_goal=goals[0][0] if goals else b
    th=hs[:,0]; v=hs[:,2]; yr=hs[:,1]; deff=eff_steer(yr,v); uu=np.interp(th,cl[:,0],cl[:,2])
    ph=np.interp(th,lp[:,0],prog); pc=np.interp(cl[:,0],lp[:,0],prog); pt=np.interp(ttr,lp[:,0],prog)
    tg=np.arange(t_on,min(t_goal,cl[-1,0]),0.1); sg=np.interp(tg,cl[:,0],cl[:,2]); hp=filtfilt(bb,ab,sg); pg=np.interp(tg,lp[:,0],prog)
    j1,j2=junctions(route,mx[lp[:,0]>=t_on],my[lp[:,0]>=t_on])
    r=dict(route=route,cfg=cfg,align_m=round(float(aerr),3),goal=goals[0][1] if goals else None,J1=j1,J2=j2,
        fails_progress=[round(float(np.interp(t,lp[:,0],prog))) for t in fl if t_on<=t<=t_goal],
        fails_before_end=sum(1 for t in fl if t_on<=t<=t_goal and np.interp(t,lp[:,0],prog)<G['end']),
        cmd_gaps=[(clock(cl[i,0]),round(x,2),round(float(pc[i]))) for i,x in enumerate(np.diff(cl[:,0])) if x>0.3 and t_on<=cl[i,0]<=t_goal],
        stale_pose_warn=Lw.count('stale pose'),neutral=[round(float(pc[i])) for i in range(1,len(cl)) if cl[i,1]==0 and cl[i,2]==0 and cl[i-1,1]>0 and t_on<cl[i,0]<t_goal and pc[i]<G['end']])
    for sn,p0,p1 in G['seg']:
        mh=(ph>=p0)&(ph<p1)&(th>=t_on)&(th<=t_goal)&(v>1); mc=(pc>=p0)&(pc<p1)&(cl[:,0]>=t_on)&(cl[:,0]<=t_goal); mt=(pt>=p0)&(pt<p1)&(tr[:,10]==1); mg=(pg>=p0)&(pg<p1)
        if mh.sum()<10 or mc.sum()<10: continue
        sc=cl[mc,2]; u=sc[np.r_[True,np.abs(np.diff(sc))>1e-6]]; dur=max(1e-3,cl[mc,0][-1]-cl[mc,0][0])
        r[sn]=dict(sat=round(float(np.mean(np.abs(sc)>=0.298)),3),hp_rms=round(float(np.degrees(np.sqrt(np.mean(hp[mg]**2)))),2),rev_s=round(float(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)/dur),2),
            yawrate_p95=round(float(np.degrees(np.quantile(np.abs(yr[mh]),.95))),1),lat_p95=round(float(np.quantile(np.abs(tr[mt,6]),.95)),2) if mt.sum() else None,
            head_p95=round(float(np.degrees(np.quantile(np.abs(tr[mt,7]),.95))),1) if mt.sum() else None,v=round(float(np.median(v[mh])),2))
    r['_plot']=(pc[(cl[:,0]>=t_on)&(cl[:,0]<=t_goal)].tolist(),np.degrees(cl[(cl[:,0]>=t_on)&(cl[:,0]<=t_goal),2]).tolist(),mx[lp[:,0]>=t_on].tolist(),my[lp[:,0]>=t_on].tolist())
    return r
F='/home/imac/ros2_ws/followup_20261006'
runs=[]
for x in json.load(open(F+'/new_runs_0342.json')):
    tag=x['bag'].split('_')[3]; S=open('/home/imac/ros2_ws/'+x['bag']+'/config_snapshot.yaml').read()
    rd=re.search(r'lower_rd_steering_rate: ([0-9.]+)',S).group(1); eps=re.search(r'wp_switch_eps: ([0-9.]+)',S); dist=re.search(r'wp_switch_max_distance_m: ([0-9.]+)',S)
    cfg=f"rdpsi{x['rdpsi'][:3]} rd{rd[:3]} wp{eps.group(1) if eps else '?'}/{dist.group(1) if dist else '?'}"
    runs.append((tag,x['route'],'/home/imac/ros2_ws/'+x['bag'],'tlog_0340.pkl',tuple(x['src']),x['yd'],x['upper'],x['lower'],cfg))
# baselines
runs.append(('N1_0137','scenario1','/home/imac/ros2_ws/drive_debug_20261006_013717_light',F+'/drive_0137/tlog_0137.pkl',(367.572235,57.392624,-1.109363),-3.107475,'upper_planner_node_25160_1791218233161.log','lower_tracking_mpc_node_25214_1791218234951.log','rdpsi3.0 rd120 wp10/15 (baseline)'))
for tag,bag,tl,src,yd,ul,ll,rd in (('S1_0141','014143',F+'/sc3_0141_0143/tlog_sc3.pkl',(229.413925,-304.748047,-1.049644),-0.521153,'upper_planner_node_27960_1791218499871.log','lower_tracking_mpc_node_27998_1791218500949.log','120'),
    ('S2_0143','014303',F+'/sc3_0141_0143/tlog_sc3.pkl',(360.707794,-270.353394,0.838671),-2.409467,'upper_planner_node_28944_1791218578855.log','lower_tracking_mpc_node_28979_1791218580435.log','120'),
    ('R1_0323','032358',F+'/sc3_0323_0328/tlog_0320.pkl',(347.580505,-216.533920,1.385218),-2.956015,'upper_planner_node_45514_1791224635438.log','lower_tracking_mpc_node_45556_1791224636497.log','120'),
    ('R2_0324','032453',F+'/sc3_0323_0328/tlog_0320.pkl',(239.940048,-88.028641,2.628966),2.083423,'upper_planner_node_46569_1791224690380.log','lower_tracking_mpc_node_46611_1791224691593.log','120'),
    ('R3_0325','032545',F+'/sc3_0323_0328/tlog_0320.pkl',(175.139343,-186.324524,-1.088879),-0.481917,'upper_planner_node_47773_1791224742803.log','lower_tracking_mpc_node_47869_1791224743914.log','120'),
    ('R4_0326','032651',F+'/sc3_0323_0328/tlog_0320.pkl',(281.981201,-148.862091,1.475190),-3.045987,'upper_planner_node_49020_1791224808611.log','lower_tracking_mpc_node_49062_1791224809673.log','120'),
    ('R5_0327','032741',F+'/sc3_0323_0328/tlog_0320.pkl',(147.224991,-23.777575,2.729237),1.983152,'upper_planner_node_49932_1791224858419.log','lower_tracking_mpc_node_49978_1791224859636.log','150'),
    ('R6_0328','032829',F+'/sc3_0323_0328/tlog_0320.pkl',(62.946770,-98.653969,-1.153243),-0.417553,'upper_planner_node_50766_1791224905756.log','lower_tracking_mpc_node_50815_1791224906753.log','150')):
    runs.append((tag,'scenario3','/home/imac/ros2_ws/drive_debug_20261006_%s_light'%bag,tl,src,yd,ul,ll,f'rdpsi3.0 rd{rd} wp10/15 (baseline)'))
OUT={}
for name,route,bag,tl,src,yd,ul,ll,cfg in runs:
    try: OUT[name]=analyze(name,route,bag,tl,src,yd,ul,ll,cfg)
    except Exception as e: print('ERR',name,e)
plots={k:v.pop('_plot') for k,v in OUT.items()}
json.dump(OUT,open('runs_generic_summary.json','w'),indent=1,default=str); pickle.dump(plots,open('runs_generic_plots.pkl','wb'))
for route in ('scenario1','scenario3'):
    print('##########',route)
    for k,r in OUT.items():
        if r['route']!=route: continue
        print(f"== {k:9s} [{r['cfg']}] goal={r['goal']} J1={r['J1']} J2={r['J2']} fails_before_end={r['fails_before_end']} fails@prog={r['fails_progress']} neutral@prog={r['neutral']} gaps={r['cmd_gaps']} stale_pose={r['stale_pose_warn']} align={r['align_m']}")
        for sn,_,_ in R[route]['seg']:
            if sn in r: print('      ',sn,r[sn])
