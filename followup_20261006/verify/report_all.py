"""Consolidated report for one result suffix (e.g. v7): synthetic regression, replica, Y synthetic, recorded map, replay."""
import json, numpy as np, sys, collections, subprocess
suf=sys.argv[1]
def steer(st):
    cmd=np.array([s['command'] for s in st]); u=cmd[np.r_[True,np.abs(np.diff(cmd))>1e-6]]
    ey=np.abs([s['ey'] for s in st if s['lower_valid']])
    return np.mean(np.abs(cmd)>=0.298), np.degrees(np.quantile(np.abs(cmd),.95)), int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)), (np.quantile(ey,.95) if len(ey) else np.nan)
print('== SYNTHETIC REGRESSION (must stay goal_reached, route_dist p95 ~2)')
for l in open(f'results_synthetic_{suf}.jsonl'):
    r=json.loads(l); st=r['states']; sat,p95,rev,ey=steer(st); rd=np.quantile([s['route_distance'] for s in st],.95)
    print(f"  {r['scene']:13s} {r['config']['name'][:20]:20s} {r['outcome']:15s} fails={sum(not p['valid'] for p in r['plans'])} sat={sat:.3f} p95={p95:.1f} rev={rev} ey95={ey:.2f} route_p95={rd:.2f}")
def agg(rows,keyf):
    a=collections.defaultdict(lambda:[0,0,0,[],[],[],[]])
    for r in rows:
        k=keyf(r); a[k][0]+=r['ok']; a[k][1]+=1; a[k][2]+=r['fails']; a[k][3].append(r['sat']); a[k][4].append(r['p95']); a[k][5].append(r['rev']); a[k][6].append(r['ey'])
    for k,v in sorted(a.items()): print(f"  {k:45s} correct {v[0]}/{v[1]} fails={v[2]} sat={np.mean(v[3]):.3f} p95={np.mean(v[4]):.1f} rev={np.mean(v[5]):.1f} ey95={np.nanmean(v[6]):.2f}")
print('== REPLICA (scenario1 J2 geometry)')
J2=np.array([121.0,-7.0]); rows=[]
for l in open(f'results_replica_{suf}.jsonl'):
    r=json.loads(l); st=r['states']; tgt='east' if 'east' in r['scene'] else 'north'; taken='none'
    for s in st:
        p=np.array([s['x'],s['y']])-J2
        if np.linalg.norm(p)>18 and p[1]>-3: taken='east' if np.degrees(np.arctan2(p[1],p[0]))<42 else 'north'; break
    sat,p95,rev,ey=steer(st); rows.append(dict(cfg=r['config']['name'],tgt=tgt,ok=taken==tgt,fails=sum(not p['valid'] for p in r['plans']),sat=sat,p95=p95,rev=rev,ey=ey))
agg(rows,lambda r:f"{r['cfg']} target={r['tgt']}")
print('== Y SYNTHETIC (straight approach, 12 cases)')
rows=[]
for l in open(f'results_ysynthetic_{suf}.jsonl'):
    r=json.loads(l); st=r['states']; name=r['scene']; geom=name.split('_')[0]; tgt=name.split('_')[2]
    J=np.array([80.0,0.0]); cont=12 if geom=='Yright' else -12; div=-40 if geom=='Yright' else 40; taken='none'
    for s in st:
        p=np.array([s['x'],s['y']])-J
        if np.linalg.norm(p)>20 and p[0]>0: ang=np.degrees(np.arctan2(p[1],p[0])); taken='diverging' if abs(ang-div)<abs(ang-cont) else 'continuing'; break
    sat,p95,rev,ey=steer(st); rows.append(dict(cfg=r['config']['name'],tgt=tgt,ok=taken==tgt,fails=sum(not p['valid'] for p in r['plans']),sat=sat,p95=p95,rev=rev,ey=ey))
agg(rows,lambda r:f"{r['cfg']} target={r['tgt']}")
print('== RECORDED aggregated map (noisy east arm), J2 east = correct')
rows=[]
for l in open(f'results_recorded_{suf}.jsonl'):
    r=json.loads(l); st=r['states']; taken='none'
    for s in st:
        p=np.array([s['x'],s['y']])-np.array([122.0,-6.0])
        if np.linalg.norm(p)>16 and p[1]>-12: taken='east' if np.degrees(np.arctan2(p[1],p[0]))<50 else 'north'; break
    sat,p95,rev,ey=steer(st); rows.append(dict(cfg=r['config']['name'],ok=taken=='east',fails=sum(not p['valid'] for p in r['plans']),sat=sat,p95=p95,rev=rev,ey=ey))
agg(rows,lambda r:r['cfg'])
print('== REAL-BEV REPLAY (open loop)'); print(subprocess.run(['python3','analyze_replay2.py'],capture_output=True,text=True).stdout)
lim=np.tan(0.30)*0.75/2.8*6.0
for tag in ('R3','A'):
    cyc=json.load(open(f'replay_{tag}.json'))['cycles']; out=json.load(open(f'replay_{tag}_out.json'))
    pre=[i for i,c in enumerate(cyc) if c['y']<-36]
    for k,rows in out.items():
        inc=[];over=0
        for r in rows:
            if not r['valid']: continue
            P=np.array(r['planned']); s_=np.diff(P,axis=0); h=np.unwrap(np.arctan2(s_[:,1],s_[:,0])); d=np.abs(np.diff(h)); inc+=list(d); over+=int(np.any(d>lim+1e-3))
        print(f"  {tag} {k:14s} fails_before_J2={sum(not rows[i]['valid'] for i in pre)}/{len(pre)} plan_increment_p95={np.degrees(np.quantile(inc,.95)):.1f}deg plans_over_steer_limit={over}")
