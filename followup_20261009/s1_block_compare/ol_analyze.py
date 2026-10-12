import json,numpy as np,collections
cyc=json.load(open('ol_cycles.json'));rec={(c['attempt'],round(c['t'],4)):c for c in cyc}
R=[json.loads(l) for l in open('ol_results.jsonl')]
def lat(old,p):
    v=np.diff(old,axis=0);ll=np.linalg.norm(v,axis=1);ll[ll<1e-9]=1e-9;u=np.clip(np.sum((p-old[:-1])*v,axis=1)/ll**2,0,1);q=old[:-1]+u[:,None]*v;j=np.argmin(np.linalg.norm(p-q,axis=1));return np.cross(v[j],p-q[j])/ll[j]
# fidelity
dB=[];dv=[];dp=[]
for r in R:
    if r['variant']!='base':continue
    for c in r['cycles']:
        q=rec[(r['attempt'],round(c['t'],4))];dB.append(abs(c['B']-q['B']));dv.append(c['valid']!=q['valid'])
        if c['valid'] and q['valid'] and q['published'] and c['published']:
            a=np.array(c['published']);b=np.array(q['published']);n=min(len(a),len(b));dp.append(np.abs(a[:n]-b[:n]).max())
print(f"FIDELITY base vs recorded: cycles={len(dB)} B mismatch>0.05: {sum(np.array(dB)>0.05)} valid mismatch: {sum(dv)} published max dev median={np.median(dp):.3f} p95={np.quantile(dp,.95):.3f} max={max(dp):.3f}")
east=lambda c:c['x']>=124 and c['x']<=165 and c['y']>-15
mouth=lambda c:110<=c['x']<124 and -26<c['y']<-5
rows=collections.defaultdict(dict)
for r in R:
    C=r['cycles'];E=[c for c in C if east(c)];M=[c for c in C if mouth(c)]
    B=np.array([c['B'] for c in E]);on=[iv[4] for c in E for iv in c['intervals'] if iv[5]==1 and iv[0]>=1];w=[iv[3] for c in E for iv in c['intervals'] if iv[5]==1 and iv[0]>=1]
    sh=[];prev=None
    for c in E:
        if c['valid'] and len(c['published'])>2:
            P=np.array(c['published'])
            if prev is not None: sh.append(lat(prev,P[2]))
            prev=P
        else: prev=None
    Bm=np.array([c['B'] for c in M])
    # B value as vehicle passes x in [112,115]
    rows[r['variant']][r['attempt']]=dict(Bmed=np.median(B),Bmin=B.min(),on=np.mean(on)*100,w=np.median(w),jit=np.sqrt(np.mean(np.square(sh))) if sh else np.nan,inv=sum(not c['valid'] for c in E),invall=sum(not c['valid'] for c in C),mouthB=[round(c['B'],1) for c in M])
V=['base','recover3','confirm','confirm+recover3','ratchet','split','split+confirm+recover3']
print('\nEAST ROAD (x124-165): B median / ON% / plan jitter rms(m) / invalid')
print('run | '+' | '.join(V))
for a in range(1,11):
    print(f"{a:2d} | "+' | '.join(f"{rows[v][a]['Bmed']:.1f}/{rows[v][a]['on']:3.0f}/{rows[v][a]['jit']:.2f}/{rows[v][a]['inv']}" for v in V))
print('\nTOTALS: mean ON% (bad runs 1,3,8) | mean jitter bad | invalid total all runs whole drive')
for v in V:
    print(f"{v:24s} ON%bad={np.mean([rows[v][a]['on'] for a in (1,3,8)]):5.1f} ON%all={np.mean([rows[v][a]['on'] for a in range(1,11)]):5.1f} jit_bad={np.mean([rows[v][a]['jit'] for a in (1,3,8)]):.3f} jit_all={np.nanmean([rows[v][a]['jit'] for a in range(1,11)]):.3f} invalid={sum(rows[v][a]['invall'] for a in range(1,11))}")
print('\nMOUTH B sequence (x110-124):')
for v in ['base','split','split+confirm+recover3']:
    for a in (1,5,10):print(f"  {v:24s} #{a}: {rows[v][a]['mouthB']}")
