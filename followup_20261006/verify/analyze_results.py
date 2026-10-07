import json, numpy as np, sys
def j1(states):
    for s in states:
        if s['y']>-38: return 'right(correct)' if s['x']>102 else 'left(wrong)'
    return 'not_reached'
def j2(states):
    J=np.array([122.0,-6.0])
    for s in states:
        p=np.array([s['x'],s['y']])
        if np.linalg.norm(p-J)>16 and p[1]>-12:
            ang=np.degrees(np.arctan2(*(p-J)[::-1]))
            return f'east(correct,{ang:.0f}deg)' if ang<50 else f'north(wrong,{ang:.0f}deg)'
    return 'not_reached'
rows=[]
for l in open(sys.argv[1]):
    r=json.loads(l); st=r['states']; pl=r['plans']
    cmd=np.array([s['command'] for s in st]); t=np.array([s['t'] for s in st])
    fails=[p for p in pl if not p['valid']]; trunc=[p for p in pl if p.get('truncated_at',-1)>0]
    u=cmd[np.r_[True,np.abs(np.diff(cmd))>1e-6]]; rev=int(np.sum(np.sign(u[1:])*np.sign(u[:-1])<0))
    ey=np.abs([s['ey'] for s in st if s['lower_valid']])
    rows.append(dict(scene=r['scene'],config=r['config']['name'],outcome=r['outcome'],t_end=round(float(t[-1]),1) if len(t) else 0,
        final=[round(v,1) for v in r['final_xy']],J1=j1(st),J2=j2(st),plans=len(pl),fails=len(fails),
        first_fail=(round(fails[0]['t'],1),[round(v,1) for v in fails[0]['pos']]) if fails else None,
        truncated_cycles=[(round(p['t'],1),p['truncated_at']) for p in trunc],
        sat=round(float(np.mean(np.abs(cmd)>=0.298)),3),cmd_p95_deg=round(float(np.degrees(np.quantile(np.abs(cmd),.95))),1),
        reversals=rev,ey_p95=round(float(np.quantile(ey,.95)),2) if len(ey) else None))
for r in rows: print(json.dumps(r))
json.dump(rows,open(sys.argv[1].replace('.jsonl','_summary.json'),'w'),indent=1)
