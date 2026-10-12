import json,sys,numpy as np,collections
for n in (2,3):
    D=f'regress/s{n}/';cyc=json.load(open(D+'ol_cycles.json'));rec={(c['attempt'],round(c['t'],4)):c for c in cyc}
    R=collections.defaultdict(dict)
    for l in open(D+'res.jsonl'):
        r=json.loads(l);R[r['variant']][r['attempt']]=r['cycles']
    fid=[abs(c['B']-rec[(a,round(c['t'],4))]['B'])>0.05 for a,C in R['base'].items() for c in C]
    fidv=[c['valid']!=rec[(a,round(c['t'],4))]['valid'] for a,C in R['base'].items() for c in C]
    print(f"\n=== scenario{n}: base fidelity B mismatch {sum(fid)}/{len(fid)}, valid mismatch {sum(fidv)}")
    for v in ['split','split+confirm+recover3']:
        nB=0;nv=0;dev=[];vb=0;vv=0;on_b=[];on_v=[];trig=[]
        for a in R['base']:
            for cb,cv in zip(R['base'][a],R[v][a]):
                nB+=abs(cb['B']-cv['B'])>0.05;vb+=not cb['valid'];vv+=not cv['valid']
                on_b+=[iv[4] for iv in cb['intervals'] if iv[5]==1];on_v+=[iv[4] for iv in cv['intervals'] if iv[5]==1]
                if cb['valid'] and cv['valid'] and len(cb['published'])>1 and len(cv['published'])>1:
                    A=np.array(cb['published']);Bp=np.array(cv['published']);m=min(len(A),len(Bp));dev.append(np.abs(A[:m]-Bp[:m]).max())
                if abs(cb['B']-cv['B'])>0.05: trig.append((a,round(cb['x']),round(cb['y'])))
        dev=np.array(dev)
        print(f"  {v:24s} B differs {nB}/{len(fid)} cycles | invalid base {vb} -> {vv} | ON% {np.mean(on_b)*100:.1f} -> {np.mean(on_v)*100:.1f} | plan dev>0.3m {np.sum(dev>0.3)} cycles, max {dev.max():.2f} m")
        locs=collections.Counter((x//10*10,y//10*10) for _,x,y in trig).most_common(4); print('     where B differs (10m cells):',locs)
