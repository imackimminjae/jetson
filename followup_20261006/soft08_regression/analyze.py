"""Per-config regression summary. Branch criteria copied from verify/report_all.py."""
import json, numpy as np, collections, sys, os
def steer(st):
    cmd=np.array([s['command'] for s in st]); u=cmd[np.r_[True,np.abs(np.diff(cmd))>1e-6]]
    ey=np.abs([s['ey'] for s in st if s['lower_valid']])
    return np.mean(np.abs(cmd)>=0.298), np.degrees(np.quantile(np.abs(cmd),.95)), int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)), (np.quantile(ey,.95) if len(ey) else np.nan)
def branch(r):
    st=r['states'];name=r['scene']
    if name.startswith('replica'):
        tgt='east' if 'east' in name else 'north'
        for s in st:
            p=np.array([s['x'],s['y']])-np.array([121.0,-7.0])
            if np.linalg.norm(p)>18 and p[1]>-3: return tgt,('east' if np.degrees(np.arctan2(p[1],p[0]))<42 else 'north')
        return tgt,'none'
    if name.startswith('Y'):
        geom=name.split('_')[0];tgt=name.split('_')[2];cont=12 if geom=='Yright' else -12;div=-40 if geom=='Yright' else 40
        for s in st:
            p=np.array([s['x'],s['y']])-np.array([80.0,0.0])
            if np.linalg.norm(p)>20 and p[0]>0: a=np.degrees(np.arctan2(p[1],p[0])); return tgt,('diverging' if abs(a-div)<abs(a-cont) else 'continuing')
        return tgt,'none'
    if name.startswith('recorded'):
        for s in st:
            p=np.array([s['x'],s['y']])-np.array([122.0,-6.0])
            if np.linalg.norm(p)>16 and p[1]>-12: return 'east',('east' if np.degrees(np.arctan2(p[1],p[0]))<50 else 'north')
        return 'east','none'
    return 'goal','goal' if r['outcome']=='goal_reached' else r['outcome']
rows=[]
for suite in ['synthetic','ysynthetic','replica','recorded','robust']:
    f=f'results_{suite}.jsonl'
    if not os.path.exists(f): continue
    for l in open(f):
        r=json.loads(l);tgt,taken=branch(r);sat,p95,rev,ey=steer(r['states'])
        rd=np.quantile([s['route_distance'] for s in r['states']],.95)
        rows.append(dict(suite=suite,scene=r['scene'],plant=r['plant']['name'],cfg=r['config']['name'],ok=tgt==taken,taken=taken,outcome=r['outcome'],
          fails=sum(not p['valid'] for p in r['plans']),nplan=len(r['plans']),sat=sat,p95=p95,rev=rev,ey=ey,rd=rd))
json.dump(rows,open('summary_rows.json','w'),indent=0)
cfgs=list(dict.fromkeys(r['cfg'] for r in rows))
for suite in dict.fromkeys(r['suite'] for r in rows):
    print(f'== {suite}')
    for c in cfgs:
        R=[r for r in rows if r['suite']==suite and r['cfg']==c]
        if not R: continue
        bad=[f"{r['scene']}|{r['plant']}:{r['taken']}/{r['outcome']}" for r in R if not r['ok']]
        print(f"  {c:16s} ok {sum(r['ok'] for r in R):2d}/{len(R):2d} planfail={sum(r['fails'] for r in R):3d}/{sum(r['nplan'] for r in R):4d} sat={np.mean([r['sat'] for r in R]):.3f} cmd95={np.mean([r['p95'] for r in R]):5.1f} rev={np.mean([r['rev'] for r in R]):5.1f} ey95={np.nanmean([r['ey'] for r in R]):.2f} route95={np.mean([r['rd'] for r in R]):.2f}")
        for b in bad: print('      BAD',b)
