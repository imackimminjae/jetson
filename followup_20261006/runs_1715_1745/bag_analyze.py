import pickle,numpy as np,sys,time
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import project
GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)
c=lambda t: time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
def junctions(mx,my):
    j1='SE' if np.any((mx>212)&(my>150)&(my<162)) and not np.any((mx<212)&(mx>195)&(my<148)) else ('SW(wrong)' if np.any((mx<212)&(mx>195)&(my<148)) else 'n/a')
    j2='NE' if np.any((mx>262)&(my>150)) else ('E(wrong)' if np.any((mx>270)&(my<140)) else 'n/a')
    return j1,j2
for tag in sys.argv[1:]:
    d=pickle.load(open(f'drive_debug_{tag}_light/analysis/decoded.pkl','rb'))
    od=d['/motive/vehicle/odom_map']; t=np.array([o['t'] for o in od]); xy=np.array([o['xy'][:2] for o in od]); v=np.array([np.hypot(*o['v'][:2]) for o in od])
    prog,lat=project(GP,xy[:,0],xy[:,1]); prog=np.asarray(prog); lat=np.asarray(lat)
    mv=v>0.5; t0=t[mv][0] if mv.any() else t[0]
    j1,j2=junctions(xy[:,0],xy[:,1])
    print(f'##### {tag}: odom {c(t[0])}~{c(t[-1])} moving from {c(t0)} max_prog={prog.max():.1f} final xy=({xy[-1,0]:.1f},{xy[-1,1]:.1f}) v_final={v[-1]:.2f} J1={j1} J2={j2} vmax={v.max():.2f}')
    ev=d['/debug/upper_branch_event']; nf=0
    for x in ev:
        e=x['data']
        if not e['valid_plan']:
            nf+=1; p=project(GP,np.array([e['x']]),np.array([e['y']]))[0][0]; print(f'   FAIL {c(x["t"])} prog={p:.1f} pos=({e["x"]:.1f},{e["y"]:.1f}) status={str(e.get("status",""))[:70]}')
    print(f'   upper cycles={len(ev)} fails={nf}')
    gr=[x for x in d['/planner/goal_reached'] if x['data']]; print('   goal_reached msgs true:',len(gr), c(gr[0]['t']) if gr else '')
    ac=d['/control/applied_cmd']; ta=np.array([a['t'] for a in ac]); s=np.array([a['angular'][2] for a in ac]); sp=np.array([a['linear'][0] for a in ac])
    print(f'   applied_cmd n={len(ac)} steer range {s.min():.3f}..{s.max():.3f} |s|>=0.98*max frac={np.mean(np.abs(s)>=0.98*np.abs(s).max()):.3f} speed cmd range {sp.min():.2f}..{sp.max():.2f}')
    dt=np.diff(ta); print(f'   cmd gaps>0.3s: {[(c(ta[i]),round(dt[i],2)) for i in np.where(dt>0.3)[0]]}')
    for seg,a,b in (('A_0_60',0,60),('B_60_115',60,115),('C_115_145',115,145)):
        m=(np.interp(ta,t,prog)>=a)&(np.interp(ta,t,prog)<b)
        if m.sum()<5: continue
        ss=s[m]; u=ss[np.r_[True,np.abs(np.diff(ss))>1e-4]]
        print(f'   {seg}: n={m.sum()} |steer| p95={np.quantile(np.abs(ss),.95):.3f} sat(>=0.98max)={np.mean(np.abs(ss)>=0.98*np.abs(s).max()):.3f} sign_flips={int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0))} route_dev_p95={np.quantile(np.abs(lat[(prog>=a)&(prog<b)]),.95):.2f}')
    for r in d['/rosout']:
        if r['level']>=30 or any(k in r['msg'] for k in ('goal','Goal','stop','arm','ARM')): print(f'   rosout {c(r["t"])} [{r["name"]}] L{r["level"]} {r["msg"][:150]}')
