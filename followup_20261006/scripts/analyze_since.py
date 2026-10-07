"""Analyze every planner launch in a time range (bag optional). Usage: analyze_since.py START END OUTDIR
Matches upper/bridge/lower ROS logs by time, extracts tlog drive windows, reports per launch:
scenario, config (from bag snapshot or upper log), progress reached, failure progress, junction outcomes,
steering metrics per route segment (end leg excluded), lower command gaps."""
import sys, os, re, glob, time, json, pickle, subprocess, numpy as np
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import eff_steer, project
from scipy.signal import butter, filtfilt
START,END,OUT=sys.argv[1],sys.argv[2],sys.argv[3]
def ts(s): return time.mktime(time.strptime(s,'%Y-%m-%d %H:%M:%S'))
c=lambda t: time.strftime('%H:%M:%S',time.localtime(t))
R={'scenario1':dict(GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float),target=(150,-140),seg=(('straight_0_88',0,88),('J1_88_125',88,125),('J2_125_150',125,150)),end=150),
   'scenario3':dict(GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float),target=(187,220),seg=(('A_0_60',0,60),('B_60_115',60,115),('C_115_145',115,145)),end=145)}
def junctions(route,mx,my):
    if route=='scenario1':
        j1=next((('right' if x>103 else 'LEFT(wrong)') for x,y in zip(mx,my) if y>-38),'n/a')
        j2='EAST' if np.any((mx>140)&(my<8)&(my>-12)) else ('NORTH(wrong)' if np.any((my>15)&(mx<135)) else 'n/a')
        return j1,j2
    j1='SE' if np.any((mx>212)&(my>150)&(my<162)) and not np.any((mx<212)&(mx>195)&(my<148)) else ('SW(wrong)' if np.any((mx<212)&(mx>195)&(my<148)) else 'n/a')
    j2='NE' if np.any((mx>262)&(my>150)) else ('E(wrong)' if np.any((mx>270)&(my<140)) else 'n/a')
    return j1,j2
logs=os.path.expanduser('~/.ros/log'); t_of=lambda n:int(n.split('_')[-1].split('.')[0])/1000
br=sorted(glob.glob(logs+'/px4_odom_map_bridge_*.log'),key=t_of); up=sorted(glob.glob(logs+'/upper_planner_node_*.log'),key=t_of); lo=sorted(glob.glob(logs+'/lower_tracking_mpc_node_*.log'),key=t_of)
bags={ts('2026-10-06 '+time.strftime('%H:%M:%S',time.strptime(os.path.basename(d).split('_')[3],'%H%M%S'))):d for d in glob.glob('/home/imac/ros2_ws/drive_debug_20261006_*')}
os.makedirs(OUT,exist_ok=True); tl=OUT+'/tlog.pkl'
if not os.path.exists(tl):
    subprocess.run(['cp','/home/imac/ros2_ws/mav.tlog',OUT+'/snap.tlog'],check=True)
    subprocess.run(['python3','/home/imac/ros2_ws/followup_20261006/tlog_extract/extract_active.py',OUT+'/snap.tlog',tl,START],check=True); os.remove(OUT+'/snap.tlog')
T=pickle.load(open(tl,'rb'))['data']
LP=np.array([(x['_t'],x['x'],x['y']) for x in T['LOCAL_POSITION_NED']]); HS=np.array([(x['_t'],x['yawspeed'],np.hypot(x['vx'],x['vy'])/100) for x in T['HIL_STATE_QUATERNION']])
CL=np.array([(x['_t'],x['param1'],x['param2']*0.35) for x in T['COMMAND_LONG'] if x['command']==187])
bb,ab=butter(2,0.3/5,'high'); rows=[]
for u in [f for f in up if ts(START)<=t_of(f)<=ts(END)]:
    tu=t_of(u); U=open(u).read()
    b=max([f for f in br if t_of(f)<=tu+3],key=t_of); B=open(b).read(); l=min(lo,key=lambda f:abs(t_of(f)-tu)); L=open(l).read()
    m=re.search(r'source=\(([-0-9.]+), ([-0-9.]+), [-0-9.]+, yaw=([-0-9.]+)\) yaw_delta=([-0-9.]+)',B); route=re.search(r'route=(scenario\d+)',B).group(1)
    if not m or route not in R: continue
    src=(float(m.group(1)),float(m.group(2))); yd=float(m.group(4)); G=R[route]
    tsl=[float(x) for x in re.findall(r'\[(\d+\.\d+)\]',U)]; t0,te=tsl[0],tsl[-1]
    fl=[float(x) for x in re.findall(r'\[(\d+\.\d+)\] \[upper_planner_node\]: egocentric MIQP failed',U)]
    goal=re.findall(r'goal reached: remaining=[0-9.]+ direct=([0-9.]+)',U)[:1]
    bag=next((d for tb,d in bags.items() if tu-15<=tb<=tu+20),None)
    if bag:
        S=open(bag+'/config_snapshot.yaml').read(); g=lambda k: (re.search(k+r': ([-0-9.]+)',S) or [None,'?'])[1]
        cfg=f"r{g('upper_r_dpsi')} rd{g('lower_rd_steering_rate')} wp{g('wp_switch_eps')}/{g('wp_switch_max_distance_m')}"
    else:
        cfg=f"r{(re.search(r'r_dpsi=([0-9.]+)',U) or [None,'?'])[1]} (no bag)"
    cfg=f"{(re.search(r'rate=([0-9.]+)Hz',U) or [None,'?'])[1]}Hz "+cfg
    mL=(LP[:,0]>=t0)&(LP[:,0]<=te+2); lp=LP[mL]
    if len(lp)<20: continue
    cc,ss=np.cos(yd),np.sin(yd); xe,ye=lp[:,2],lp[:,1]; tx,ty=G['target']
    mx=tx+cc*(xe-src[0])-ss*(ye-src[1]); my=ty+ss*(xe-src[0])+cc*(ye-src[1]); prog,lat=project(G['GP'],mx,my)
    mc=(CL[:,0]>=t0)&(CL[:,0]<=te); cl=CL[mc]
    if not (cl[:,1]>0).any(): continue
    t_on=cl[cl[:,1]>0][0,0]; mv=lp[:,0]>=t_on
    j1,j2=junctions(route,mx[mv],my[mv])
    fp=[round(float(np.interp(t,lp[:,0],prog))) for t in fl]
    gaps=[(c(cl[i,0]),round(x,2)) for i,x in enumerate(np.diff(cl[:,0])) if x>0.3 and cl[i,0]>=t_on]
    r=dict(start=c(tu),route=route,cfg=cfg,bag=os.path.basename(bag) if bag else None,max_prog=round(float(prog[mv].max())),goal=goal[0] if goal else None,J1=j1,J2=j2,fails_progress=fp,
           fails_before_end=sum(1 for p in fp if p<G['end']),cmd_gaps=gaps,stale_pose=L.count('stale pose') if abs(t_of(l)-tu)<10 else '?')
    th=HS[:,0]; mh0=(th>=t_on)&(th<=te)
    ph=np.interp(th,lp[:,0],prog); pc=np.interp(cl[:,0],lp[:,0],prog)
    tg=np.arange(t_on,te,0.1); sg=np.interp(tg,cl[:,0],cl[:,2]); hp=filtfilt(bb,ab,sg) if len(sg)>20 else sg*0; pg=np.interp(tg,lp[:,0],prog)
    for sn,p0,p1 in G['seg']:
        mh=mh0&(ph>=p0)&(ph<p1)&(HS[:,2]>1); mcc=(pc>=p0)&(pc<p1)&(cl[:,0]>=t_on); mg=(pg>=p0)&(pg<p1)
        if mh.sum()<10 or mcc.sum()<10: continue
        sc=cl[mcc,2]
        r[sn]=dict(sat=round(float(np.mean(np.abs(sc)>=0.298)),3),hp_rms=round(float(np.degrees(np.sqrt(np.mean(hp[mg]**2)))),2),yawrate_p95=round(float(np.degrees(np.quantile(np.abs(HS[mh,1]),.95))),1),route_dev_p95=round(float(np.quantile(np.abs(np.interp(th[mh],lp[:,0],lat)),.95)),2))
    rows.append(r)
json.dump(rows,open(OUT+'/summary.json','w'),indent=1)
for r in rows:
    print(f"{r['start']} {r['route']} [{r['cfg']}] bag={r['bag']} max_prog={r['max_prog']} goal={r['goal']} J1={r['J1']} J2={r['J2']} fails@prog={r['fails_progress']} gaps={r['cmd_gaps']}")
    for sn in [s[0] for s in R[r['route']]['seg']]:
        if sn in r: print('      ',sn,r[sn])
