from pathlib import Path
import json,gzip,numpy as np
H=Path(__file__).resolve().parent
load=lambda p:json.loads(gzip.decompress(p.read_bytes())) if p.suffix=='.gz' else json.loads(p.read_text())
def dist(a,b):
 a=np.array(a);b=np.array(b)
 return float(np.max(np.linalg.norm(a-b,axis=1))) if a.shape==b.shape and len(a) else None
summary={};validation={};key=[];shift=[]
for man in H.glob('manifest_*.json'):
 for item in load(man):
  tag=item['tag'];inp=load(Path(item['input']));cs=inp['cycles'];res=load(H/f'result_{tag}.json.gz');summary[tag]={};val=[]
  recorded=res.get('recorded_fixed',res.get('base_fixed'))
  for j,(c,z) in enumerate(zip(cs,recorded)):
   err=dist(z.get('planned',[]),c['rec_planned']) if z['valid'] and c['rec_valid'] else None
   val.append(dict(index=j,t=c['t'],valid_match=z['valid']==c['rec_valid'],plan_error_m=err,B_error=abs(z['B']-c['rec_B'])))
  validation[tag]=dict(n=len(cs),valid_matches=sum(x['valid_match'] for x in val),comparable=sum(x['plan_error_m'] is not None for x in val),within_1mm=sum(x['plan_error_m'] is not None and x['plan_error_m']<.001 for x in val),max_plan_error_m=max((x['plan_error_m'] for x in val if x['plan_error_m'] is not None),default=0),mismatches=[x for x in val if not x['valid_match'] or x['plan_error_m'] is not None and x['plan_error_m']>.001])
  for name,zs in res.items():
   use=[z for c,z in zip(cs,zs) if c.get('running',True)];jumps=[];prev=None
   for c,z in zip(cs,zs):
    p=z.get('published',z.get('planned',[]))
    if len(p)>1:
     if prev is not None and c['t']-prev[0]<.8:
      a=np.array(prev[1]);v=np.diff(a,axis=0);ll=(v*v).sum(axis=1);u=np.clip(((np.array(p[1])-a[:-1])*v).sum(axis=1)/np.maximum(ll,1e-12),0,1);q=a[:-1]+u[:,None]*v;i=int(np.argmin(np.linalg.norm(p[1]-q,axis=1)));d=float(np.cross(v[i],p[1]-q[i])/max(np.sqrt(ll[i]),1e-12));jumps.append(abs(d))
      if tag.startswith('batch') and 224<c['x']<229:shift.append(dict(tag=tag,mode=name,t=c['t'],x=c['x'],jump=d,B_before=z['B_before'],B=z['B'],k1=p[1]))
     prev=(c['t'],p)
   summary[tag][name]=dict(n=len(use),fail=sum(not z['valid'] for z in use),guard=sum(z['side_boundary_tail_start']>=0 for z in use),k1_shift_p95=float(np.quantile(jumps,.95)) if jumps else None)
  if tag.startswith('batch'):
   for j,c in enumerate(cs):
    if (230<c['x']<251 and int(tag[-2:]) in [1,4,5,6]) or (250<c['x']<273 and tag=='batch07'):
     row=dict(tag=tag,index=j,t=c['t'],x=c['x'],y=c['y'],recorded=c['rec_planned'][-1:])
     row['modes']={name:dict(valid=zs[j]['valid'],B=zs[j]['B'],end=zs[j].get('planned',[])[-1:],k1=zs[j].get('planned',[])[1:2],counts=zs[j]['counts'],guard=zs[j]['side_boundary_tail_start']) for name,zs in res.items()};key.append(row)
(H/'summary.json').write_text(json.dumps(summary,indent=2));(H/'validation.json').write_text(json.dumps(validation,indent=2));(H/'key_events.json').write_text(json.dumps(key,indent=2));(H/'k1_shifts.json').write_text(json.dumps(shift,indent=2))
for tag,z in summary.items():print(tag,'valid reproduction',validation[tag]['within_1mm'],validation[tag]['comparable'],'fails', {n:v['fail'] for n,v in z.items()})
