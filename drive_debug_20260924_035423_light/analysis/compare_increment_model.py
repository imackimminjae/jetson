from pathlib import Path
from collections import Counter
import json,numpy as np
root=Path(__file__).resolve().parent
wrap=lambda a:np.arctan2(np.sin(a),np.cos(a))
metrics={}
for label in ['increment_legacy','increment_fixed','increment_no_turn','increment_turn_03']:
 z=json.loads((root/f'replay_{label}.json').read_text()); offset=z['wall_to_bag_offset']
 a=np.array(z['lower']);t=a[:,0]-offset;m=(t>=1790189701)&(t<1790189718)&(a[:,10]==1);u=np.degrees(a[m,8])
 ht=[];head=[];errors=[];branch_errors=[];corrections=[]
 traces=np.array([x[0] for x in z['upper']])
 for e in z['events']:
  if not e['valid_plan']:continue
  pts=np.array(e['planned_world']);segments=np.diff(pts,axis=0)
  head.append(np.arctan2(segments[0,1],segments[0,0]));ht.append(e['t_start']-offset)
  idx=int(np.argmin(abs(traces-e['t_emit'])))
  if abs(traces[idx]-e['t_emit'])>.05:continue
  tr=z['upper'][idx];N=int(tr[7]);off=18+3*(N+1)
  model=np.array(tr[off+N:off+2*N]);actual=wrap(np.arctan2(segments[:,1],segments[:,0])-e['yaw_rad'])
  error=float(max(abs(np.degrees(wrap(actual-model)))))
  errors.append(error)
  if tr[14]>=1:branch_errors.append(error)
 head=np.degrees(np.unwrap(head));ht=np.array(ht);hm=(ht>=1790189701)&(ht<1790189718)
 metrics[label]=dict(upper_outcomes=dict(Counter(e['reason'] for e in z['events'])),horizons=dict(Counter(e['horizon'] for e in z['events'])),lower_status=dict(Counter(str(x[12]) for x in z['lower'])),steering_rms_deg=float(np.sqrt(np.mean(u*u))),steering_total_variation_deg=float(sum(abs(np.diff(u)))),heading_total_variation_deg=float(sum(abs(np.diff(head[hm])))),heading_jump_max_deg=float(max(abs(np.diff(head[hm])))),saturation_fraction=float(np.mean(abs(u)>17.1)),heading_mismatch_max_deg=max(errors),branch_heading_mismatch_median_deg=float(np.median(branch_errors)),branch_heading_mismatch_p95_deg=float(np.percentile(branch_errors,95)),branch_heading_mismatch_max_deg=max(branch_errors))
(root/'increment_comparison.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(2,1,figsize=(11,6),sharex=True)
zero=1790189700
for label,title in [('increment_legacy','Before'),('increment_fixed','Consistent increments')]:
 z=json.loads((root/f'replay_{label}.json').read_text());offset=z['wall_to_bag_offset'];a=np.array(z['lower'])
 ev=[e for e in z['events'] if e['valid_plan']];pts=[np.array(e['planned_world']) for e in ev]
 h=np.unwrap([np.arctan2((p[1]-p[0])[1],(p[1]-p[0])[0]) for p in pts]);t=[e['t_start']-offset-zero for e in ev]
 axs[0].plot(t,np.degrees(h),label=title)
 axs[1].plot(a[:,0]-offset-zero,np.degrees(a[:,8]),label=title)
axs[0].set_ylabel('First path heading (deg)');axs[1].set_ylabel('Steering command (deg)');axs[1].set_xlabel('Seconds since 03:55:00 KST (recording time)')
axs[0].set_title('Same recorded inputs; offline replay, not closed-loop driving')
for ax in axs:ax.grid(alpha=.3);ax.legend();ax.set_xlim(0,24)
fig.tight_layout();fig.savefig(root/'increment_comparison.png',dpi=150)
