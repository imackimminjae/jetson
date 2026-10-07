import json, numpy as np, sys
rows=[]
for l in open(sys.argv[1]):
    r=json.loads(l); st=r['states']; pl=r['plans']; inp=None
    # scene meta via name
    name=r['scene']; geom=name.split('_')[0]; target=name.split('_')[2]
    J=np.array([80.0,0.0]); cont=12 if geom=='Yright' else -12; div=-40 if geom=='Yright' else 40
    taken='not_reached'
    for s in st:
        p=np.array([s['x'],s['y']])-J
        if np.linalg.norm(p)>20 and p[0]>0:
            ang=np.degrees(np.arctan2(p[1],p[0])); taken='diverging' if abs(ang-div)<abs(ang-cont) else 'continuing'; break
    cmd=np.array([s['command'] for s in st]); u=cmd[np.r_[True,np.abs(np.diff(cmd))>1e-6]]
    fails=[round(p['t'],1) for p in pl if not p['valid']]
    rows.append(dict(scene=name,config=r['config']['name'],target=target,taken=taken,correct=taken==target,outcome=r['outcome'],fails=fails,
        trunc=[(round(p['t'],1),p['truncated_at']) for p in pl if p.get('truncated_at',-1)>0],
        sat=round(float(np.mean(np.abs(cmd)>=0.298)),3),cmd_p95=round(float(np.degrees(np.quantile(np.abs(cmd),.95))),1),
        rev=int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0)),ey_p95=round(float(np.quantile(np.abs([s['ey'] for s in st if s['lower_valid']]),.95)),2)))
json.dump(rows,open(sys.argv[1].replace('.jsonl','_summary.json'),'w'),indent=1)
for r in rows: print(f"{r['scene']:40s} {r['config'][:9]:9s} target={r['target'][:4]} taken={r['taken'][:4]:4s} {'OK ' if r['correct'] else 'BAD'} {r['outcome'][:10]:10s} fails={r['fails']} trunc={len(r['trunc'])} sat={r['sat']} p95={r['cmd_p95']} rev={r['rev']} ey95={r['ey_p95']}")
import collections
agg=collections.defaultdict(lambda:[0,0,0])
for r in rows: a=agg[(r['config'],r['target'])]; a[0]+=r['correct']; a[1]+=1; a[2]+=len(r['fails'])
print({f'{k[0]}|{k[1]}':f'{v[0]}/{v[1]} correct, fails={v[2]}' for k,v in agg.items()})
