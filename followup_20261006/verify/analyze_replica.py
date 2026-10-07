import json, numpy as np, sys, collections
J2=np.array([121.0,-7.0])
rows=[]
for l in open(sys.argv[1]):
    r=json.loads(l); st=r['states']; pl=r['plans']; name=r['scene']; target='east' if 'east' in name else 'north'
    taken='not_reached'
    for s in st:
        p=np.array([s['x'],s['y']])-J2
        if np.linalg.norm(p)>18 and p[1]>-3:
            ang=np.degrees(np.arctan2(p[1],p[0])); taken='east' if ang<42 else 'north'; break
    cmd=np.array([s['command'] for s in st]); u=cmd[np.r_[True,np.abs(np.diff(cmd))>1e-6]]
    rows.append(dict(scene=name,config=r['config']['name'],target=target,taken=taken,correct=taken==target,outcome=r['outcome'],
        fails=[round(p['t'],1) for p in pl if not p['valid']],trunc=[(round(p['t'],1),p['truncated_at']) for p in pl if p.get('truncated_at',-1)>0],
        branch_steps=[p['branch'] for p in pl],sat=round(float(np.mean(np.abs(cmd)>=0.298)),3),cmd_p95=round(float(np.degrees(np.quantile(np.abs(cmd),.95))),1),
        rev=int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)),final=[round(v,1) for v in r['final_xy']]))
json.dump(rows,open(sys.argv[1].replace('.jsonl','_summary.json'),'w'),indent=1)
for r in rows: print(f"{r['scene']:36s} {r['config'][:9]:9s} target={r['target']:5s} taken={r['taken']:5s} {'OK ' if r['correct'] else 'BAD'} {r['outcome'][:12]:12s} final={r['final']} fails={r['fails']} trunc={r['trunc']} br={r['branch_steps']} sat={r['sat']} p95={r['cmd_p95']} rev={r['rev']}")
agg=collections.defaultdict(lambda:[0,0,0])
for r in rows: a=agg[(r['config'],r['target'])]; a[0]+=r['correct']; a[1]+=1; a[2]+=len(r['fails'])
print({f'{k[0]}|{k[1]}':f'{v[0]}/{v[1]} correct, fails={v[2]}' for k,v in agg.items()})
