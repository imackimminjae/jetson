from pathlib import Path
import json,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
H=Path(__file__).resolve().parent;s=json.loads((H/'summary.json').read_text());es=json.loads((H/'events.json').read_text());ph=s['phases'];a=next(x['t'] for x in ph if x['phase']=='running');b=next(x['t'] for x in ph if x['phase']=='coasting');r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=s['session']+'/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/debug/upper_interval_centering_state','/debug/lower_mpc_trace']))
bs=[];ls=[]
while r.has_next():
 n,blob,t=r.read_next()
 if not a<=t/1e9<b:continue
 v=list(deserialize_message(blob,Float64MultiArray).data)
 (bs if n.endswith('centering_state') else ls).append(v)
bs=np.array(bs);ls=np.array(ls);ev=[e for e in es if a<=e['t_start']<b];prev=None;shifts=[]
for e in ev:
 if not e['valid_plan']:continue
 plan=np.array(e['published_world'])
 if len(plan)<2:continue
 if prev is not None and 0<e['t_start']-prev['t_start']<.8:
  old=np.array(prev['published_world']);v=np.diff(old,axis=0);ll=np.linalg.norm(v,axis=1);u=np.clip(np.sum((plan[1]-old[:-1])*v,axis=1)/np.maximum(ll**2,1e-12),0,1);q=old[:-1]+u[:,None]*v;j=int(np.argmin(np.linalg.norm(plan[1]-q,axis=1)));shift=float(np.cross(v[j],plan[1]-q[j])/max(ll[j],1e-12));bi=int(np.argmin(abs(bs[:,0]-e['t_start'])))
  shifts.append(dict(t=e['t_start'],xy=[e['x'],e['y']],k1_lateral_m=shift,B_before=float(bs[bi,1]),B=float(bs[bi,2]),horizon=e['horizon']))
 prev=e
active=ls[(ls[:,12]==0)&(ls[:,4]>1.)];sig=active[abs(active[:,26])>np.deg2rad(1)];flips=sum(np.sign(x[26])!=np.sign(y[26]) for x,y in zip(sig[:-1],sig[1:]) if 0<y[0]-x[0]<.7)
stops=ls[(ls[:,12]==4)&(ls[:,0]>a+2)]; first_stop=stops[0]; stopped=np.array([293.418,198.1142]);
out=dict(first_neutral_stamp=float(first_stop[0]),first_neutral_xy=first_stop[1:3].tolist(),first_neutral_speed_mps=float(first_stop[4]),neutral_to_final_stop_displacement_m=float(np.linalg.norm(stopped-first_stop[1:3])),run_start=a,run_end=b,B_range=[float(np.nanmin(bs[:,2])),float(np.nanmax(bs[:,2]))],largest_k1_shifts=sorted(shifts,key=lambda x:abs(x['k1_lateral_m']),reverse=True)[:5],steering_sign_flips_over_1deg=int(flips),post_reset_plan_events=sum(e['t_start']>=next(x['t'] for x in ph if x['index']==2 and x['phase']=='reference') for e in es),all_guard_hits=sum(e['side_boundary_tail_start']>=0 for e in es),all_path_revisions=sorted(set(e['path_revision'] for e in es)),second_to_tenth_wp_pairs=sorted(set((e['wp0'],e['wp1']) for e in es if e['t_start']>=next(x['t'] for x in ph if x['index']==2 and x['phase']=='reference'))))
(H/'details.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2));print('first',json.dumps(s['attempts'][0],indent=2))
