import json, numpy as np, time
def clock(t): return time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
J2=np.array([121.0,-7.0])
def arm(P):
    if P is None: return 'FAIL'
    P=np.array(P); end=P[-1]-J2
    if np.linalg.norm(end)<6: return 'mouth'
    ang=np.degrees(np.arctan2(end[1],end[0])); return f"{'EAST' if ang<43 else 'nth'}{ang:.0f}"
for tag in ('R3','A'):
    cyc=json.load(open(f'replay_{tag}.json'))['cycles']; out=json.load(open(f'replay_{tag}_out.json'))
    keys=list(out.keys()); print('=====',tag,'runs:',keys)
    for i,c in enumerate(cyc):
        if c['y']<-36 or c['y']>12: continue
        print(clock(c['t']),f"({c['x']:.0f},{c['y']:.0f})",' '.join(f"{arm(out[k][i].get('planned') if out[k][i]['valid'] else None):8s}" for k in keys))
    print('fail counts (all cycles):',{k:sum(not r['valid'] for r in out[k]) for k in keys})
