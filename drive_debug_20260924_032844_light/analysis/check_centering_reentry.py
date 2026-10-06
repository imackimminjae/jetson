from pathlib import Path
import pickle,json,numpy as np
from datetime import datetime
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'));events=[x['data'] for x in d['/debug/upper_branch_event']];details=[x['data'].reshape(-1,12) for x in d['/debug/upper_interval_info']];results=[]
for i in range(1,len(events)):
 e,old=events[i],events[i-1]
 if not e['valid_plan'] or not old['valid_plan']:continue
 for inter in e['intervals_world']:
  k,j=inter[:2]
  if len(e['selected_corridors'])<=k or e['selected_corridors'][k]!=j:continue
  now=next(r for r in details[i] if r[0]==k and r[1]==j);before=next((r for r in details[i-1] if r[0]==k and r[1]==j),None)
  if before is None or before[4] or not now[4]:continue
  a,b=np.array(inter[2:4]),np.array(inter[4:6]);axis=(b-a)/np.linalg.norm(b-a);tan=np.array([axis[1],-axis[0]]);ref=np.array(e['preview_world'][k]);oldpath=np.array(old['planned_world']);cross=[]
  for p0,p1 in zip(oldpath[:-1],oldpath[1:]):
   denom=tan@(p1-p0)
   if abs(denom)<1e-9:continue
   alpha=tan@(ref-p0)/denom
   if 0<=alpha<=1:cross.append(p0+alpha*(p1-p0))
  if not cross:continue
  p=min(cross,key=lambda p:np.linalg.norm(p-ref));length=inter[7];coord=float(axis@(p-a));extra=.5*(inter[6]-length);current=float(axis@(np.array(e['planned_world'][k])-a))
  record=dict(time=datetime.fromtimestamp(e['t_start']).strftime('%H:%M:%S.%f')[:-3],seq=e['plan_seq'],k=k,old_raw=float(before[2]),old_processed=float(before[3]),raw=inter[6],processed=length,B=now[10],old_path_outside_processed_m=max(0.,-coord,coord-length),old_path_inside_raw=bool(-extra<=coord<=length+extra),current_path_boundary_distance_m=min(current,length-current),current_vs_old_lateral_shift_m=current-coord)
  results.append(record);print(record)
(root/'centering_reentry.json').write_text(json.dumps(results,indent=2))
