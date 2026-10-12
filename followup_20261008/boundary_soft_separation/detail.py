from pathlib import Path
import json,gzip,numpy as np
from datetime import datetime
from zoneinfo import ZoneInfo
H=Path(__file__).resolve().parent
load=lambda p:json.loads(gzip.decompress(p.read_bytes())) if p.suffix=='.gz' else json.loads(p.read_text())
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
rows=[];shifts=[];guard=[]
for i in range(1,11):
 tag=f'batch{i:02d}';cs=load(H/f'input_{tag}.json')['cycles'];r=load(H/f'result_{tag}.json.gz');q=load(H/f'refine_{tag}.json.gz')
 for j,(c,a,b) in enumerate(zip(cs,r['base_fixed'],r['hold_update'])):
  if 224<c['x']<228 and c['rec_B']-c['rec_B_before']>1:
   prev=np.array(cs[j-1]['rec_published']);v=np.diff(prev,axis=0);ll=(v*v).sum(axis=1)
   def shift(z):
    p=np.array(z['planned'][1]);u=np.clip(((p-prev[:-1])*v).sum(axis=1)/ll,0,1);pt=prev[:-1]+u[:,None]*v;n=int(np.argmin(np.linalg.norm(p-pt,axis=1)));return float(np.cross(v[n],p-pt[n])/np.sqrt(ll[n]))
   changed=[dict(k=x['k'],i=x['i'],raw=x['raw'],before_width=y['processed'],after_width=x['processed'],before_soft=y['soft'],after_soft=x['soft']) for x,y in zip(a['intervals'],b['intervals']) if x['soft']!=y['soft']]
   shifts.append(dict(attempt=i,time=clock(c['t']),index=j,xy=[c['x'],c['y']],B_before=c['rec_B_before'],B=c['rec_B'],same_old_plan_shift_updated_B=shift(a),same_old_plan_shift_previous_B=shift(b),direct_k1_change_m=float(np.linalg.norm(np.array(a['planned'][1])-np.array(b['planned'][1]))),changed_intervals=changed,turn_steps_updated=a['turn_steps'],turn_steps_previous=b['turn_steps']))
  if a['side_boundary_tail_start']>=0:
   guard.append(dict(attempt=i,time=clock(c['t']),on=a.get('planned',[])[-1:],off_fixed=r['guard_off_fixed'][j].get('planned',[])[-1:],off_carried=r['guard_off_carried'][j].get('planned',[])[-1:]))
  if i==5 and 231<c['x']<241:
   modes={}
   for name in ['base_fixed','edge_fixed','edge_carried','edge_B_fixed','edge_ref_fixed','gated_skip_carried','clip_margin_fixed','clip_raw_fixed']:
    z=(q if name in q else r)[name][j]
    modes[name]=dict(valid=z['valid'],B=z['B'],end=z.get('planned',[])[-1:],k1=z.get('planned',[])[1:2],ref=z['reference'],k6=[d for d in z['intervals'] if d['k']==6],raw_k6=[d for d in z['raw_detail'] if d['k']==6],k6_cands=z['cands'][-1],turn_steps=z['turn_steps'])
   rows.append(dict(time=clock(c['t']),index=j,xy=[c['x'],c['y']],yaw=c['yaw'],modes=modes))
(H/'B_jump_causal.json').write_text(json.dumps(shifts,indent=2));(H/'clipped_direction.json').write_text(json.dumps(rows,indent=2));(H/'guard_ablation.json').write_text(json.dumps(guard,indent=2))
for x in shifts:print(x['attempt'],x['time'],round(x['same_old_plan_shift_updated_B'],3),round(x['same_old_plan_shift_previous_B'],3),'changed k',[d['k'] for d in x['changed_intervals']],'turn',x['turn_steps_updated'],x['turn_steps_previous'])
