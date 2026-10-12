"""Compare old/new geometry at equal progress from the current ego projection."""
from pathlib import Path
import pickle,json
import numpy as np
from datetime import datetime
p=Path(__file__).resolve().parent
d=pickle.load(open(p.parent/'drive_debug_20261001_184521_light/analysis/decoded.pkl','rb'))
def arc(points):return np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
def projection(points,point):
 seg=np.diff(points,axis=0);u=np.clip(np.sum((point-points[:-1])*seg,axis=1)/np.sum(seg*seg,axis=1),0,1)
 foot=points[:-1]+u[:,None]*seg;idx=np.argmin(np.linalg.norm(foot-point,axis=1));s=arc(points)
 return s[idx]+u[idx]*(s[idx+1]-s[idx])
def sample(points,q):
 s=arc(points);idx=np.clip(np.searchsorted(s,q,side='right')-1,0,len(points)-2)
 a=(q-s[idx])/(s[idx+1]-s[idx]);delta=points[idx+1]-points[idx]
 return points[idx]+a*delta,np.arctan2(delta[1],delta[0])
rows=[];previous=None;oldbranch=None
for z in d['/debug/upper_branch_event']:
 e=z['data']
 if not e['valid_plan']:continue
 path=np.array(e['planned_world']);pose=np.array([e['x'],e['y']]);tr=min(d['/debug/upper_miqp_trace'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data']
 branch=int(tr[14]);row={'time':datetime.fromtimestamp(e['t_start']).strftime('%H:%M:%S.%f')[:-3],'t':e['t_emit'],'branch':branch,'turn_cost_active':branch>=1,'wp':[e['wp0'],e['wp1']]}
 if previous is not None:
  progress=projection(previous,pose)
  row['branch_changed']=branch!=oldbranch
  for distance in [5,10,15]:
   if progress+distance>arc(previous)[-1] or distance>arc(path)[-1]:continue
   old,h0=sample(previous,progress+distance);new,h1=sample(path,distance)
   row[str(distance)]={'distance_m':float(np.linalg.norm(new-old)),'lateral_m':float((new-old)@[-np.sin(h0),np.cos(h0)]),'heading_change_deg':float(np.degrees(np.arctan2(np.sin(h1-h0),np.cos(h1-h0))))}
 rows.append(row);previous=path;oldbranch=branch
(p/'aligned_changes.json').write_text(json.dumps(rows,indent=2))
for row in sorted([r for r in rows if '5'in r],key=lambda r:abs(r['5']['heading_change_deg']),reverse=True)[:15]:print(row)
