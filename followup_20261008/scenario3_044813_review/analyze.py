from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from collections import Counter
import json,pickle,hashlib,numpy as np,yaml
from scipy.signal import butter,sosfiltfilt
H=Path(__file__).resolve().parent;R=H.parents[1]
clock=lambda t:datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def project(xy,route):
 v=np.diff(route,axis=0);l=np.linalg.norm(v,axis=1)
 u=np.clip(np.sum((xy[:,None,:]-route[None,:-1,:])*v,axis=2)/l**2,0,1)
 q=route[None,:-1,:]+u[:,:,None]*v;dist=np.linalg.norm(xy[:,None,:]-q,axis=2);j=np.argmin(dist,axis=1);ii=np.arange(len(xy));arc=np.r_[0,np.cumsum(l)]
 return arc[j]+u[ii,j]*l[j],dist[ii,j]
def quant(a):
 return dict(zip(['min','median','p95','max'],map(float,np.quantile(a,[0,.5,.95,1])))) if len(a) else None
data={};out={};hashes={}
for ident in ['20261008_044813']:
 p=R/f'drive_debug_{ident}_light';d=pickle.load((p/'analysis/decoded.pkl').open('rb'));data[ident]=d
 es=[z['data'] for z in d['/debug/upper_branch_event']];tr=np.array([z['data'] for z in d['/debug/lower_mpc_trace']]);od=d['/motive/vehicle/odom_map'];xy=np.array([z['xy'] for z in od]);route=d['/navigation/global_path'][0]['xy'];prog,dist=project(xy,route);lp,ld=project(tr[:,1:3],route);active=(tr[:,12]==0)&(tr[:,4]>1.)
 shifts=[];prev=None
 for i,e in enumerate(es):
  if not e['valid_plan']:continue
  plan=np.array(e['planned_world'])
  if prev is not None and 0<e['t_start']-prev[0]<.8:
   vv=np.diff(prev[1],axis=0);ll=np.linalg.norm(vv,axis=1);u=np.clip(np.sum((plan[1]-prev[1][:-1])*vv,axis=1)/np.maximum(ll**2,1e-12),0,1);qq=prev[1][:-1]+u[:,None]*vv;j=int(np.argmin(np.linalg.norm(plan[1]-qq,axis=1)));shift=float(np.cross(vv[j],plan[1]-qq[j])/max(ll[j],1e-12));pr=float(project(np.array([[e['x'],e['y']]]),route)[0][0]);shifts.append(dict(i=i,time=clock(e['t_start']),t=e['t_start'],progress=pr,lateral_m=shift))
  prev=e['t_start'],plan
 hp=np.full(len(tr),np.nan)
 # Filter each contiguous active stretch, never across startup/restart/goal gaps.
 ii=np.flatnonzero(active);splits=np.flatnonzero((np.diff(ii)>1)|(np.diff(tr[ii,0])>.3))+1
 for chunk in np.split(ii,splits):
  if len(chunk)<20:continue
  tm=np.arange(tr[chunk[0],0],tr[chunk[-1],0],.1)
  if len(tm)<20:continue
  yy=np.interp(tm,tr[chunk,0],tr[chunk,26]);ff=sosfiltfilt(butter(2,.3,fs=10,btype='highpass',output='sos'),yy)
  hp[chunk]=np.interp(tr[chunk,0],tm,ff)
 segments={}
 for name,lo,hi in [('early',0,53.3),('middle',53.3,103.3),('last_turn',103.3,151.9),('goal_approach',151.9,200),('whole',0,200)]:
  m=active&(lp>=lo)&(lp<hi);z=tr[m];sg=[]
  for i in np.flatnonzero(m):
   if abs(tr[i,26])>np.deg2rad(1):sg.append((tr[i,0],int(np.sign(tr[i,26]))))
  changes=sum(a[1]!=b[1] for a,b in zip(sg[:-1],sg[1:]) if b[0]-a[0]<.7)
  hs=hp[m];hs=hs[np.isfinite(hs)];ss=[abs(s['lateral_m']) for s in shifts if lo<=s['progress']<hi]
  segments[name]=dict(n=len(z),speed=quant(z[:,4]),lateral_abs=quant(abs(z[:,6])),heading_abs_deg=quant(abs(np.rad2deg(z[:,7]))),sat_fraction=float(np.mean(abs(z[:,26])>=.29845)) if len(z) else None,steering_highpass_rms_deg=float(np.sqrt(np.mean(np.rad2deg(hs)**2))) if len(hs) else None,steering_sign_flips_over_1deg=changes,k1_shift_abs=quant(ss))
 grids=d['/bev/occupancy_grid'];near_rows=[]
 for g in grids:
  a=g['grid'];xx=g['origin'][0]+(np.arange(g['width'])+.5)*g['res'];yy=g['origin'][1]+(np.arange(g['height'])+.5)*g['res'];near=(xx>=0)&(xx<2.95);central=(abs(yy)<=2)
  mask=central[:,None]&near[None,:];v=a[mask];near_rows.append((g['t'],int(np.sum(v==0)),int(np.sum(v<0)),int(v.size)))
 goals=[z for z in d.get('/planner/goal_reached',[]) if z['data']]
 goal=None
 if goals:
  t=goals[0]['t'];j=int(np.argmin(abs(np.array([z['t'] for z in od])-t)));goal=dict(time=clock(t),t=t,xy=xy[j].tolist(),direct_distance_m=float(np.linalg.norm(xy[j]-route[-1])),projected_remaining_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()-prog[j]))
 row=dict(bag=str(p),pose_window=[clock(od[0]['t']),clock(od[-1]['t'])],route=route.tolist(),upper_count=len(es),upper_valid=sum(e['valid_plan'] for e in es),upper_horizons=dict(Counter(e['horizon'] for e in es)),k1_exclusion_true_count=sum(any(e.get('k1_lane_constraint_excluded',[])) for e in es),lower_modes=dict(Counter(map(int,tr[:,12]))),goal=goal,last_xy=xy[-1].tolist(),last_goal_distance_m=float(np.linalg.norm(xy[-1]-route[-1])),grid_count=len(grids),grid_max_gap_s=float(np.max(np.diff([g['t'] for g in grids]))),grid_geometry_patterns=dict(Counter(str((g['width'],g['height'],g['res'],g['origin'])) for g in grids)),near_center_free_range=[min(z[1] for z in near_rows),max(z[1] for z in near_rows)],near_center_cell_counts=sorted(set(z[3] for z in near_rows)),near_center_all_free_frames=sum(z[1]==z[3] for z in near_rows),near_center_any_unknown_frames=sum(z[2]>0 for z in near_rows),segments=segments,largest_k1_shifts=sorted(shifts,key=lambda s:abs(s['lateral_m']),reverse=True)[:8])
 out[ident]=row
 if ident=='20261008_044813':
  (H/'latest_plan_shifts.json').write_text(json.dumps(shifts,indent=2));(H/'rosout.txt').write_text('\n'.join(clock(z['t'])+' '+z['name']+' '+z['msg'] for z in d['/rosout']))
 for f in [p/'bag/bag_0.db3',p/'bag/metadata.yaml',p/'config_snapshot.yaml']:
  if f.exists():hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()

(H/'summary.json').write_text(json.dumps(out,indent=2));(H/'sources_sha256.json').write_text(json.dumps(hashes,indent=2))
print(json.dumps(out,indent=2))
