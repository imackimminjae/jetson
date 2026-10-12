from pathlib import Path
import pickle,json,numpy as np,sys,itertools
from scipy.optimize import linprog
from datetime import datetime
from zoneinfo import ZoneInfo
H=Path(__file__).resolve().parent;R=H.parents[1];sys.path.insert(0,str(R/'followup_20261006/reference_B_separation'));from feasibility import constraints
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
rows=[]
for tag,ii in [('044813',25),('045057',27),('045146',33)]:
 d=pickle.load(open(R/f'drive_debug_20261008_{tag}_light/analysis/decoded.pkl','rb'));e=d['/debug/upper_branch_event'][ii]['data'];cs=np.array([z['data'] for z in d['/debug/upper_interval_centering_state']]);s=cs[np.argmin(abs(cs[:,0]-e['t_start']))];iv=[q for q in e['intervals_world'] if q[0]==1][0];a=np.array(iv[2:4]);b=np.array(iv[4:6]);p=np.array(e['preview_world'][1]);u=(b-a)/np.linalg.norm(b-a);mid=(a+b)/2;oldlen=iv[6]*(.4) if iv[6]<=1.5*s[1]+1e-9 else iv[6]-4.;oa=mid-oldlen*u/2;ob=mid+oldlen*u/2
 distance=lambda p,a,b:float(np.linalg.norm(p-(a+np.clip(np.dot(p-a,b-a)/np.dot(b-a,b-a),0,1)*(b-a))))
 rows.append(dict(tag=tag,event=ii,time=clock(e['t_start']),xy=[e['x'],e['y']],B_before=s[1],B_after=s[2],raw_width=iv[6],actual_width=iv[7],old_B_width=oldlen,reference_outside_actual_m=distance(p,a,b),reference_outside_old_B_m=distance(p,oa,ob)))
(H/'B_repeat.json').write_text(json.dumps(rows,indent=2));print('B_REPEAT',json.dumps(rows))
d=pickle.load(open(R/'drive_debug_20261008_045057_light/analysis/decoded.pkl','rb'));out=[]
for i in [35,36,37,54]:
 e=d['/debug/upper_branch_event'][i]['data'];N=e['horizon'];c={'yaw':e['yaw_rad']};r={'reference':e['preview_world'],'cands':[[] for _ in range(N+1)],'counts':e['candidate_counts']}
 for iv in e['intervals_world']:r['cands'][iv[0]].append(iv[2:6])
 for sel in itertools.product(*(range(n) for n in r['counts'][1:])):
  A,b=constraints(r,c,sel)
  for mode in ['full','omit_last_lane']:
   inds=list(range(len(b))) if mode=='full' else list(range(2*N-2))+list(range(2*N,len(b)))
   z=linprog(np.zeros(2*N),A_ub=A[inds],b_ub=b[inds],bounds=[(None,None)]*(2*N),method='highs')
   out.append(dict(event=i,time=clock(e['t_start']),mode=mode,sel=sel,feasible=bool(z.success)))
(H/'independent_feasibility.json').write_text(json.dumps(out,indent=2));print('LP',json.dumps(out))
extra=[]
for tag in ['044813','045057','045146']:
 d=pickle.load(open(R/f'drive_debug_20261008_{tag}_light/analysis/decoded.pkl','rb'));od=d['/motive/vehicle/odom_map'];xy=np.array([z['xy'] for z in od]);ot=np.array([z['t'] for z in od]);goal=d['/navigation/global_path'][0]['xy'][-1];dist=np.linalg.norm(xy-goal,axis=1);tr=np.array([z['data'] for z in d['/debug/lower_mpc_trace']]);goals=[z['t'] for z in d.get('/planner/goal_reached',[]) if z['data']];tg=goals[0] if goals else None
 extra.append(dict(tag=tag,closest_goal_m=float(min(dist)),closest_time=clock(ot[np.argmin(dist)]),last_speed_mps=float(tr[-1,4]),post_goal_record_sec=float(ot[-1]-tg) if tg else None,last_goal_dist_m=float(dist[-1]),goal_speed_mps=float(tr[np.argmin(abs(tr[:,0]-tg)),4]) if tg else None,route_messages=len(d['/navigation/global_path']),route_patterns=len(set(str(z['xy']) for z in d['/navigation/global_path'])),config_equal_to_first=(R/f'drive_debug_20261008_{tag}_light/config_snapshot.yaml').read_bytes()==(R/'drive_debug_20261008_044813_light/config_snapshot.yaml').read_bytes()))
(H/'details.json').write_text(json.dumps(extra,indent=2));print('DETAILS',json.dumps(extra))
