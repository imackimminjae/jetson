from pathlib import Path
import pickle,json,itertools,numpy as np
from scipy.optimize import linprog
from datetime import datetime
p=Path(__file__).resolve().parent;d=pickle.load(open(p.parent/'drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'));events=[x['data'] for x in d['/debug/upper_branch_event']];details=d['/debug/upper_interval_info'];features=json.load(open(p.parent/'drive_debug_20261003_055920_light/analysis/features.json'))
def solve(e,mode='current',ratio=.7,steer=.3,slack=False):
 N=e['horizon'];tr=min(d['/debug/upper_miqp_trace'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data'];h=tr[18:18+N+1];inf=min(details,key=lambda x:abs(x['t']-e['t_emit']))['data'].reshape(-1,12);yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xy=np.array([e['x'],e['y']]);ref=(np.array(e['preview_world'])-xy)@R
 n=2*N;dim=n+int(slack);G=np.zeros((N+1,2,dim));nom=np.zeros(dim)
 for k in range(N):nom[2*k+1]=np.arctan2(np.sin(h[k]-(h[k-1] if k else 0)),np.cos(h[k]-(h[k-1] if k else 0)))
 for k in range(1,N+1):
  for j in range(k):G[k,:,2*j]=.75*np.array([np.cos(h[j]),np.sin(h[j])]);G[k,:,2*j+1]=4.5*np.array([-np.sin(h[j]),np.cos(h[j])])
 A=[];b=[];phi=np.tan(steer)*.75/2.8
 for k in range(N):
  v=np.zeros(dim);v[2*k]=1
  for sign in [1,-1]:r=sign*v;A.append(r);b.append(.75-r@nom)
  for sign in [1,-1]:
   r=np.zeros(dim);r[:2*k+1:2]=-phi;r[2*k+1]=sign;A.append(r);b.append(phi*6-r@nom)
 options=[];clipped=0
 for k in range(1,N+1):
  opts=[];axis=np.array([-np.sin(h[k]),np.cos(h[k])]);row=axis@G[k]
  for inter in e['intervals_world']:
   if int(inter[0])!=k:continue
   ir=next(v for v in inf if int(v[0])==k and int(v[1])==int(inter[1]));raw=ir[2];on=bool(ir[4]);clipped+=int(ir[7]);end=(np.array(inter[2:6]).reshape(2,2)-xy)@R;mid=end.mean(axis=0);dire=end[1]-end[0];dire/=np.linalg.norm(dire);inset=(raw-np.linalg.norm(end[1]-end[0]))/2
   if mode=='ratio' and on:inset=(1-ratio)*raw
   if mode=='soft_off' and on:inset=min(2,.45*raw)
   if mode=='no_soft' and on:inset=0
   if mode=='no_margin' and not on:inset=0
   if mode=='raw':inset=0
   if mode=='first_raw' and k==1:inset=0
   if mode=='freeze_B':
    i=next(i for i,v in enumerate(events) if v is e);B=features[max(0,i-1)]['B'];inset=(1-.7)*raw if raw<=1.5*B else min(2,.45*raw)
   ends=np.array([mid+(-raw/2+inset)*dire,mid+(raw/2-inset)*dire])
   if mode=='clipped_boundary' and ir[7]:
    inset=min(2,.45*raw);ends=np.array([mid+(-raw/2+inset)*dire,mid+(raw/2-inset)*dire])
   if mode in ['clip_keep','clipped_boundary']:
    if not ir[5]:ends[0]=mid-raw/2*dire
    if not ir[6]:ends[1]=mid+raw/2*dire
   lo,hi=sorted(ends@axis);proj=ref[k]@axis;rr=np.array([row,-row]);bb=np.array([hi-proj,proj-lo])
   if slack:rr[:,-1]=-1
   opts.append((rr,bb))
  options.append(opts)
 best=None
 for combo in itertools.product(*options):
  aa=np.vstack([A]+[x[0] for x in combo]);bb=np.concatenate([b]+[x[1] for x in combo]);c=np.zeros(dim)
  if slack:c[-1]=1
  z=linprog(c,A_ub=aa,b_ub=bb,bounds=[(None,None)]*n+([(0,None)] if slack else []),method='highs')
  if z.success:
   if not slack:return True
   best=z.fun if best is None else min(best,z.fun)
  elif z.status!=2:raise RuntimeError(z.message)
 return best if slack else False
out=[]
for e in events:
 z=solve(e);assert z==e['valid_plan'],(e['t_start'],z,e['reason'])
 if z:continue
 row={'time':datetime.fromtimestamp(e['t_start']).strftime('%H:%M:%S'),'minimum_uniform_boundary_relaxation_m':solve(e,slack=True)}
 for mode in ['no_soft','soft_off','no_margin','clip_keep','clipped_boundary','freeze_B','first_raw','raw']:row[mode]=solve(e,mode)
 row['ratio_results']={str(r):solve(e,'ratio',ratio=r) for r in [.75,.8,.85,.9,.95,1.]}
 row['steer_results']={str(s):solve(e,steer=s) for s in [.35,.4,.5]}
 out.append(row);print(row,flush=True)
(p/'constraint_audit.json').write_text(json.dumps({'all_36_outcomes_reproduced':True,'cases':out},indent=2))
