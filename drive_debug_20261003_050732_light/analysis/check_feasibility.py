from pathlib import Path
import pickle,json,itertools,numpy as np
from scipy.optimize import linprog
from datetime import datetime
p=Path(__file__).resolve().parent;d=pickle.load(open(p/'decoded.pkl','rb'));clock=lambda t:datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
def reconstruct(e,mode='actual',prefix=None):
 tr=min(d['/debug/upper_miqp_trace'],key=lambda r:abs(r['data'][0]-e['t_emit']))['data'];assert abs(tr[0]-e['t_emit'])<.05
 n=e['horizon'] if prefix is None else prefix;N=e['horizon'];h=tr[18:18+N+1];yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xy=np.array([e['x'],e['y']]);ref=(np.array(e['preview_world'])-xy)@R
 G=np.zeros((n+1,2,2*n));nom=np.zeros(2*n)
 for k in range(n):
  nom[2*k+1]=np.arctan2(np.sin(h[k]-(0 if k==0 else h[k-1])),np.cos(h[k]-(0 if k==0 else h[k-1])))
 for k in range(1,n+1):
  for j in range(k):G[k,:,2*j]=.75*np.array([np.cos(h[j]),np.sin(h[j])]);G[k,:,2*j+1]=4.5*np.array([-np.sin(h[j]),np.cos(h[j])])
 phi=np.tan(.3)*.75/2.8*(2 if mode=='double_turn_bound' else 1);A=[];b=[]
 for k in range(n):
  r=np.zeros(2*n);r[2*k]=1
  for rr in [r,-r]:A.append(rr);b.append(.75-rr@nom)
  for sign in [1,-1]:
   r=np.zeros(2*n);r[:2*k+1:2]=-phi;r[2*k+1]=sign;A.append(r);b.append(phi*6-r@nom)
 lanes=[]
 for k in range(1,n+1):
  opts=[];axis=np.array([-np.sin(h[k]),np.cos(h[k])]);row=axis@G[k]
  for inter in e['intervals_world']:
   if int(inter[0])!=k:continue
   ends=(np.array(inter[2:6]).reshape(2,2)-xy)@R;mid=np.mean(ends,axis=0);dire=(ends[1]-ends[0]);dire/=np.linalg.norm(dire);width=np.linalg.norm(ends[1]-ends[0]);raw=inter[6]
   if mode=='ratio_08' and abs(width-.4*raw)<1e-5:width=.6*raw
   if mode=='raw':width=raw
   if mode=='ratio_09':width=.8*raw
   if mode=='margin_half':width=(raw+width)/2
   ends=np.array([mid-width*.5*dire,mid+width*.5*dire]);lo,hi=sorted(ends@axis);proj=ref[k]@axis;opts.append(([row,-row],[hi-proj,proj-lo]))
  lanes.append(opts)
 for combo in itertools.product(*lanes):
  AA=np.vstack([A]+[o[0] for o in combo]);bb=np.concatenate([b]+[o[1] for o in combo])
  r=linprog(np.zeros(2*n),A_ub=AA,b_ub=bb,bounds=[(None,None)]*(2*n),method='highs')
  if r.success:return True
  assert r.status==2,(r.status,r.message)
 return False
rows=[]
for e in [r['data'] for r in d['/debug/upper_branch_event']]:
 if e['reason']!='solver_rejected':continue
 row={'time':clock(e['t_start']),'t':e['t_start']}
 for mode in ['actual','ratio_08','raw']:row[mode]=reconstruct(e,mode)
 row['first_infeasible_prefix']=next((n for n in range(1,e['horizon']+1) if not reconstruct(e,prefix=n)),None);rows.append(row);print(row,flush=True)
# Cross-check successful controls, including boundaries adjacent to the failures.
ev=[r['data'] for r in d['/debug/upper_branch_event']];indices={j for i,e in enumerate(ev) if e['reason']=='solver_rejected' for j in (i-1,i+1) if 0<=j<len(ev) and ev[j]['reason']=='success'}
checks=[{'time':clock(ev[i]['t_start']),'feasible':reconstruct(ev[i])} for i in sorted(indices)];print('success_checks',checks,flush=True)
(p/'feasibility_results.json').write_text(json.dumps({'failures':rows,'success_checks':checks},indent=2))
