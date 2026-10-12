from pathlib import Path
from collections import Counter
import json,gzip,csv,os,hashlib
import numpy as np
H=Path(__file__).resolve().parent;R=H.parents[1];P=H.parent/'scenario1_sway_offline'
S=R/'followup_20261008/batches_182344_183952_185426_review/scenario1'
d=json.load(gzip.open(S/'telemetry.json.gz','rt'))['data'];info=d['/debug/upper_interval_info'];it=np.array([t for t,_ in info]);bs=np.array([v for _,v in d['/debug/upper_interval_centering_state']])
def rebuild(prev,c):
    p=np.array(prev);pos=np.array([c['x'],c['y']]);v=np.diff(p,axis=0);ll=np.linalg.norm(v,axis=1);arc=np.r_[0,np.cumsum(ll)]
    u=np.clip(np.sum((pos-p[:-1])*v,axis=1)/ll**2,0,1);dist=np.linalg.norm(pos-(p[:-1]+u[:,None]*v),axis=1);j=np.argmin(dist);progress=arc[j]+u[j]*ll[j]
    N=len(c['rec_preview'])-1;q=[pos]
    for k in range(1,N-1):
        s=progress+4.5*k
        if s>=arc[-1]:w=p[-1]+(s-arc[-1])*v[-1]/ll[-1]
        else:
            b=np.searchsorted(arc,s,side='right');w=p[b-1]+(s-arc[b-1])/ll[b-1]*v[b-1]
        q.append(w)
    vec=np.diff(np.array(q),axis=0)[-3:];direction=np.sum(vec/np.linalg.norm(vec,axis=1)[:,None],axis=0);direction/=np.linalg.norm(direction)
    q.append(q[-1]+4.5*direction);q.append(q[-1]+4.5*direction)
    return np.array(q),float(progress),int(j)
rows=[];audits=[];key=[];mismatches=[];reconstruction=[]
for n in range(1,11):
    cs=json.loads((P/f'input_{n}.json').read_text())['cycles'];rr=json.load(gzip.open(P/f'upper_{n}.json.gz','rt'))['fixed'];prev=None;prev_t=None
    for j,(c,replay) in enumerate(zip(cs,rr)):
        iv=np.array(info[np.argmin(abs(it-c['t']))][1]).reshape(-1,12);b=bs[np.argmin(abs(bs[:,0]-c['t']))]
        eligible=[];obs=[]
        for v in iv:
            k,i=int(v[0]),int(v[1]);inside_count=int(sum((iv[:,0]==k)&(iv[:,9]>0)))
            reason='eligible' if inside_count==1 and bool(v[9]) and bool(v[5]) and bool(v[6]) and not bool(v[8]) else ('reference_outside' if not v[9] else 'reference_ambiguous' if inside_count!=1 else 'fallback' if v[8] else 'boundary_unobserved')
            q=dict(k=k,i=i,raw=float(v[2]),processed=float(v[3]),inside=bool(v[9]),observed=[bool(v[5]),bool(v[6])],reason=reason)
            if reason=='eligible':eligible.append(q)
            obs.append(q)
        winner=min(eligible,key=lambda q:q['raw']) if eligible else None
        predicted=float(winner['raw']) if winner and (b[1]<=0 or winner['raw']<=1.5*b[1]) else float(b[1])
        ok=abs(predicted-b[2])<1e-7 and (not winner or (abs(winner['raw']-b[4])<1e-7 and winner['k']==int(b[5]) and winner['i']==int(b[6])))
        if not ok:mismatches.append([n,j,predicted,b.tolist()])
        record=dict(attempt=n,cycle=j,t=c['t'],x=c['x'],y=c['y'],yaw_deg=float(np.degrees(c['yaw'])),B_before=float(b[1]),B=float(b[2]),candidate=float(b[4]) if np.isfinite(b[4]) else None,candidate_k=int(b[5]),candidate_i=int(b[6]),eligible_count=len(eligible),accepted=winner is not None and abs(predicted-winner['raw'])<1e-7,observations=obs,preview=c['rec_preview'])
        if prev is not None:
            ref,progress,segment=rebuild(prev,c);error=float(np.max(np.linalg.norm(ref-c['rec_preview'],axis=1)));record.update(reference_error=error,previous_t=prev_t,previous_progress=progress,previous_segment=segment)
            reconstruction.append(dict(attempt=n,cycle=j,error=error))
        if c['rec_valid']:prev=c['rec_planned'];prev_t=c['t']
        audits.append(record)
        rows.append({k:v for k,v in record.items() if k not in ['observations','preview']})
    group=[z for z in audits if z['attempt']==n and -30<z['y']<-14]
    high=next(z for z in group if z['B']>=13);drop=next(z for z in group if z['t']>high['t'] and z['B']<10)
    before_drop=next(z for z in audits if z['attempt']==n and z['cycle']==drop['cycle']-1)
    q=min(group,key=lambda z:abs(z['y']+22.5))
    key.append(dict(attempt=n,high=high,drop=drop,previous=before_drop,critical=q,high_duration_sec=drop['t']-high['t']))
(H/'audit.json').write_text(json.dumps(dict(source=str(S),cycles=len(audits),B_mismatches=mismatches,reference_reconstruction=reconstruction,key=key,cycles_detail=audits),indent=2))
with (H/'B_history.csv').open('w') as f:
    fields=list(dict.fromkeys(k for r in rows for k in r));w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
print('audit',len(audits),'B mismatch',len(mismatches),'reference max',max(z['error'] for z in reconstruction),'ref>1mm',sum(z['error']>.001 for z in reconstruction))
for k in key:print(k['attempt'],'high',k['high']['B'],k['high']['candidate_k'],round(k['high']['y'],2),'drop',k['drop']['B'],k['drop']['candidate_k'],round(k['drop']['y'],2),'duration',round(k['high_duration_sec'],2))
assert not mismatches
