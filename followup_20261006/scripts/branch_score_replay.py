"""Offline check: at each cycle with >=2 corridor candidates, which candidate does a simple route score pick
(distance from candidate center to the remaining global route polyline from wp0 onward) vs what the planner selected."""
import pickle, numpy as np, sys
from analyze_runs import clock, ROUTES
GP=ROUTES['scenario1']
def dist_to_route(p, wp0):
    pts=GP[wp0-1 if wp0>0 else 0:]  # include incoming segment end
    best=1e9
    for a,b in zip(pts[:-1],pts[1:]):
        d=b-a; u=np.clip((p-a)@d/(d@d),0,1); best=min(best,np.linalg.norm(p-(a+u*d)))
    return best
for name,p in (('A 05:59','/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl'),('R3 23:45','/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis/decoded.pkl')):
    d=pickle.load(open(p,'rb')); print('=====',name)
    for x in d['/debug/upper_branch_event']:
        e=x['data']; cands={}
        for it in e['intervals_world']:
            cands.setdefault(int(it[0]),[]).append((int(it[1]),(np.array(it[2:4])+np.array(it[4:6]))/2))
        multi=[k for k,v in cands.items() if len(v)>=2]
        if not multi: continue
        k=max(multi); opts=cands[k]
        scores=[(j,round(dist_to_route(c,e['wp0']),1)) for j,c in opts]
        pick=min(scores,key=lambda s:s[1])[0]
        sel=e['selected_corridors'][k] if e['valid_plan'] and e['selected_corridors'] else None
        flag='' if sel is None else ('agree' if sel==pick else 'DISAGREE')
        print(clock(x['t']),'pos',np.round([e['x'],e['y']],1),'wp',e['wp0'],e['wp1'],'k=%d'%k,'route_dist(idx,m)=',scores,'route_pick',pick,'planner_sel',sel,flag)
