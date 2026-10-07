import pickle, numpy as np, time, sys
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import project
ROUTE=sys.argv[1]
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float) if ROUTE=='scenario1' else np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)
J=np.array([122.0,-6.0]) if ROUTE=='scenario1' else np.array([257.0,139.0])
LO,HI=(95,150) if ROUTE=='scenario1' else (85,135)
def clock(t): return time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
for tag in sys.argv[2:]:
    d=pickle.load(open(f'/home/imac/ros2_ws/drive_debug_20261006_{tag}_light/analysis/decoded.pkl','rb'))
    info=d['/debug/upper_interval_info']; cs=d['/debug/upper_interval_centering_state']
    print('#######',tag)
    for x in d['/debug/upper_branch_event']:
        e=x['data']; p=np.array([e['x'],e['y']]); prog,_=project(GP,np.array([p[0]]),np.array([p[1]])); prog=float(prog[0])
        if not (LO<=prog<=HI): continue
        inf=min(info,key=lambda r:abs(r['t']-x['t']))['data'].reshape(-1,12); s=min(cs,key=lambda r:abs(r['data'][0]-e['t_emit']))['data']
        end='-'
        if e['valid_plan']:
            P=np.array(e['planned_world']); v=P[-1]-J; ang=np.degrees(np.arctan2(v[1],v[0])); end=('EAST' if ang<43 else 'north')+'(%.0f)'%ang if ROUTE=='scenario1' else ('NE' if 25<ang<90 else 'E/other')+'(%.0f)'%ang
        steps=[]
        for k in range(1,e['horizon']+1):
            rows=inf[inf[:,0]==k]
            steps.append('k%d:'%k+'|'.join('%.1f%s%s'%(r[2],'s' if r[4] else 'm','c' if r[7] else '') for r in rows))
        print(f"{clock(x['t'])} prog={prog:5.1f} pos=({p[0]:.0f},{p[1]:.0f}) yaw={np.degrees(e['yaw_rad']):4.0f} valid={str(e['valid_plan'])[0]} B={s[2]:.1f} thr={1.5*s[2]:.1f} br={e['branch_step']:2d} sel={e['selected_corridors']} end={end}")
        print('        ',' '.join(steps))
