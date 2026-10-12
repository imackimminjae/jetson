from pathlib import Path
import pickle,numpy as np
H=Path(__file__).resolve().parent;R=H.parents[1]
for tag in ['20261008_042308','20261008_043557','20261008_044813','20261008_045057','20261008_045146','20261006_050627']:
 d=pickle.load(open(R/f'drive_debug_{tag}_light/analysis/decoded.pkl','rb'));hits=[]
 for i,ev in enumerate(d['/debug/upper_branch_event']):
  e=ev['data'];N=e['horizon'];r=np.array(e['preview_world']);yaw=e['yaw_rad'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);p=np.array([e['x'],e['y']]);rb=(r-p)@rot
  if N<2:continue
  intervals={k:[] for k in [N-1,N]}
  for v in e['intervals_world']:
   k=v[0]
   if k not in intervals:continue
   ab=np.array(v[2:6]).reshape(2,2);u=(ab[1]-ab[0])/np.linalg.norm(ab[1]-ab[0]);mid=ab.mean(axis=0);raw=np.array([mid-.5*v[6]*u,mid+.5*v[6]*u]);body=(raw-p)@rot;projection=(r[k]-raw[0])@u;inside=projection>=-1e-8 and projection<=v[6]+1e-8
   intervals[k].append((inside,body))
  if any(inside for inside,body in intervals[N]):continue
  for side in [-1,1]:
   clip=lambda b: np.any(abs(b[:,1]-side*20)<.11)
   if any(inside and clip(b) for inside,b in intervals[N-1]) and not any(clip(b) for inside,b in intervals[N]) and 20-side*rb[N,1]<=np.linalg.norm(rb[N]-rb[N-1])+.3125 and side*(rb[N,1]-rb[N-1,1])>0:
    hits.append((i,round(e['x'],2),round(e['y'],2),side))
 print(tag,hits)
