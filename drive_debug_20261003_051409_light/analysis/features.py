from pathlib import Path
from datetime import datetime
import pickle,json,numpy as np
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'))
wrap=lambda x:np.arctan2(np.sin(x),np.cos(x));clock=lambda t:datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
rows=[];previous=None
for evt in d['/debug/upper_branch_event']:
 tr=min(d['/debug/upper_miqp_trace'], key=lambda r:abs(r['data'][0]-evt['data']['t_emit']))
 ss=min(d['/debug/upper_interval_centering_state'], key=lambda r:abs(r['data'][0]-tr['data'][0]))
 detail=min(d['/debug/upper_interval_info'], key=lambda r:abs(r['t']-evt['t']))
 assert abs(ss['data'][0]-tr['data'][0])<.05 and abs(detail['t']-evt['t'])<.05
 e=evt['data'];a=tr['data'];s=ss['data'];infos=detail['data'].reshape(-1,12);assert abs(a[0]-e['t_emit'])<.1
 r=dict(t=float(a[0]),clock=clock(a[0]),seq=e['plan_seq'],valid=e['valid_plan'],x=e['x'],y=e['y'],yaw_deg=np.degrees(e['yaw_rad']),wp=[e['wp0'],e['wp1']],branch=int(a[14]),B=float(s[2]),candidateB=float(s[4]),Bstep=int(s[5]),Bindex=int(s[6]),Jpos=float(a[15]),Jturn=float(a[17]),road_delta_deg=np.degrees(a[13]),intervals=[])
 p=np.array(e['planned_world']);ref=np.array(e['preview_world'])
 if len(p)>1:
  h=np.arctan2(*(p[1]-p[0])[::-1]);hr=np.arctan2(*(ref[1]-ref[0])[::-1]);r.update(h1_deg=np.degrees(h),refh1_deg=np.degrees(hr),innovation_deg=np.degrees(wrap(h-hr)),h1_jump_deg=np.degrees(wrap(h-previous)) if previous is not None else 0);previous=h
 for inter in e['intervals_world']:
  k,j=int(inter[0]),int(inter[1]);ir=next(v for v in infos if v[0]==k and v[1]==j);selected=e['valid_plan'] and e['selected_corridors'][k]==j;lo=np.array(inter[2:4]);hi=np.array(inter[4:6]);unit=(hi-lo)/np.linalg.norm(hi-lo)
  margin=min((p[k]-lo)@unit,(hi-p[k])@unit) if selected else None
  r['intervals'].append(dict(k=k,j=j,raw=ir[2],processed=ir[3],on=bool(ir[4]),contains_reference=bool(ir[9]),selected=selected,margin=margin,center=((lo+hi)/2).tolist(),endpoints=[lo.tolist(),hi.tolist()]))
 rows.append(r)
(root/'features.json').write_text(json.dumps(rows,indent=2))
for r in rows:
 if r['clock']<'04:56:20':continue
 ints=[i for i in r['intervals'] if i['selected'] and i['k']==1];near=ints[0] if ints else {}
 print(r['clock'],r['valid'],'wp',r['wp'],'br',r['branch'],'B',round(r['B'],2),'cand',round(r['candidateB'],2),'k',r['Bstep'],'yaw/h/ref',*[round(r.get(k,float('nan')),1) for k in ['yaw_deg','h1_deg','refh1_deg']],'jump',round(r.get('h1_jump_deg',0),1),'k1', {k:round(v,2) if isinstance(v,float) else v for k,v in near.items() if k in ['raw','processed','on','margin']})
