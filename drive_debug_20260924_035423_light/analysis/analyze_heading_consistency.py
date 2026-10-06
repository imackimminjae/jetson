from pathlib import Path
from datetime import datetime
import json,pickle,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'));wrap=lambda a:np.arctan2(np.sin(a),np.cos(a));records=[]
for event,item in zip(d['/debug/upper_branch_event'],d['/debug/upper_miqp_trace']):
 e=event['data'];a=item['data'];N=int(a[7]);assert abs(e['t_emit']-a[0])<.1
 off=18+3*(N+1);model=a[off+N:off+2*N];p=np.array(e['planned_world']);segments=np.diff(p,axis=0);actual=wrap(np.arctan2(segments[:,1],segments[:,0])-e['yaw_rad']);err=np.degrees(wrap(actual-model));records.append(dict(t=float(a[0]),branch=int(a[14]),max_mismatch_deg=float(max(abs(err))),geometry_heading_deg=np.degrees(actual).tolist(),model_heading_deg=np.degrees(model).tolist()))
(root/'heading_consistency.json').write_text(json.dumps(records,indent=2))
metrics={};fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True);zero=1790189700
rr=np.array([[x['t'],x['branch'],x['max_mismatch_deg']] for x in records]);axs[0].plot(rr[:,0]-zero,rr[:,2],color='tab:red');axs[0].set_ylabel('Heading mismatch (deg)');axs[0].set_title('Upper predicted heading vs direction of published path segments')
for label,title in [('baseline','Turn cost enabled'),('no_turn','Turn cost disabled (diagnostic only)')]:
 z=json.loads((root/f'replay_{label}.json').read_text());a=np.array(z['lower']);t=a[:,0]-z['wall_to_bag_offset'];m=(t>=1790189701)&(t<1790189718)&(a[:,10]==1);u=np.degrees(a[m,8]);head=[];ht=[]
 for e in z['events']:
  if not e['valid_plan']:continue
  p=np.array(e['planned_world']);v=p[1]-p[0];head.append(np.arctan2(v[1],v[0]));ht.append(e['t_start']-z['wall_to_bag_offset'])
 head=np.degrees(np.unwrap(head));ht=np.array(ht);hm=(ht>=1790189701)&(ht<1790189718)
 metrics[label]=dict(steering_rms_deg=float(np.sqrt(np.mean(u*u))),steering_total_variation_deg=float(sum(abs(np.diff(u)))),heading_total_variation_deg=float(sum(abs(np.diff(head[hm])))),heading_jump_max_deg=float(max(abs(np.diff(head[hm])))),saturation_fraction=float(np.mean(abs(u)>17.1)))
 axs[1].plot(ht-zero,head,label=title);axs[2].plot(t-zero,np.degrees(a[:,8]),label=title)
axs[1].set_ylabel('First path heading (deg)');axs[2].set_ylabel('Steering (deg)');axs[2].set_xlabel('Seconds since 03:55:00 (KST)')
for ax in axs:ax.set_xlim(0,24);ax.grid(alpha=.3)
axs[1].legend();axs[2].legend();fig.tight_layout();fig.savefig(root/'branch_heading_diagnosis.png',dpi=150)
(root/'replay_comparison.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
