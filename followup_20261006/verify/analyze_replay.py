import json, numpy as np, time
def clock(t): return time.strftime('%H:%M:%S',time.localtime(t))+('%.1f'%(t%1))[1:]
J2=np.array([121.0,-7.0]); E=np.array([18.0]); 
def arm(planned):
    """classify planned path end relative to J2: east arm direction ~18 deg, north ~68 deg."""
    if planned is None: return '-'
    P=np.array(planned); end=P[-1]-J2
    if np.linalg.norm(end)<6: return 'mouth'
    ang=np.degrees(np.arctan2(end[1],end[0])); return f"{'EAST' if ang<43 else 'north'}({ang:.0f})"
for tag in ('R3','A'):
    cyc=json.load(open(f'replay_{tag}.json'))['cycles']; out=json.load(open(f'replay_{tag}_out.json'))
    print('=====',tag,'  columns: time pos | recorded(valid,branch,end-arm) | replay production | replay V3')
    agree=0;n=0
    for c,p0,p3 in zip(cyc,out['0'],out['3']):
        pos=np.array([c['x'],c['y']])
        if pos[1]<-60 or pos[1]>12: continue
        rec=arm(c['rec_planned'] if c['rec_valid'] else None)
        a0=arm(p0.get('planned')); a3=arm(p3.get('planned'))
        print(f"{clock(c['t'])} ({pos[0]:.0f},{pos[1]:.0f}) | rec {str(c['rec_valid'])[0]} br={c['rec_branch']:2d} {rec:11s} | prod {str(p0['valid'])[0]} br={p0['branch']:2d} {a0:11s} | V3 {str(p3['valid'])[0]} br={p3['branch']:2d} {a3:11s}")
