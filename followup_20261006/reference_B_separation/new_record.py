from pathlib import Path
import pickle,json,numpy as np
from scipy.signal import butter,sosfiltfilt
here=Path(__file__).resolve().parent;root=here.parents[1]
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
def progress(xy):
 v=np.diff(GP,axis=0);l=np.linalg.norm(v,axis=1);d=xy[:,None,:]-GP[:-1];u=np.clip(np.sum(d*v,axis=2)/l**2,0,1);q=GP[:-1]+u[:,:,None]*v;j=np.argmin(np.sum((q-xy[:,None,:])**2,axis=2),axis=1)
 return np.r_[0,np.cumsum(l)][j]+u[np.arange(len(xy)),j]*l[j]
out={}
for tag in ['050627','055105']:
 d=pickle.load((root/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));a=np.array([r['data'] for r in d['/debug/lower_mpc_trace']]);a=a[np.argsort(a[:,0])];p=progress(a[:,1:3]);t=np.arange(a[0,0],a[-1,0],.1);cmd=np.degrees(np.interp(t,a[:,0],a[:,26]));hp=sosfiltfilt(butter(2,.3,fs=10,btype='high',output='sos'),cmd);tp=np.interp(t,a[:,0],p);events=[r['data'] for r in d['/debug/upper_branch_event']]
 summary={}
 for name,lo,hi in [('J1',88,125),('J2',125,150),('exit',150,190)]:
  m=(p>=lo)&(p<hi)&(a[:,10]==1);mt=(tp>=lo)&(tp<hi)
  summary[name]=dict(samples=int(sum(m)),trace_command_saturation_fraction=float(np.mean(abs(a[m,26])>=.298)),trace_local_lateral_p95_m=float(np.quantile(abs(a[m,6]),.95)),trace_command_highpass_rms_deg=float(np.sqrt(np.mean(hp[mt]**2))),trace_speed_median_mps=float(np.median(a[m,4])),sign_changes_above_1deg=int(np.sum(np.diff(np.sign(a[m,26][abs(a[m,26])>np.radians(1)]))!=0)))
 od=np.array([r['xy'] for r in d['/motive/vehicle/odom_map']]);out[tag]=dict(segments=summary,recorded_upper_failures=sum(not e['valid_plan'] for e in events),recorded_events=len(events),last_xy=od[-1].tolist(),last_distance_to_goal=float(np.linalg.norm(od[-1]-GP[-1])),goal_message_any_true=any(r['data'] for r in d['/planner/goal_reached']))
(here/'new_record_summary.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
