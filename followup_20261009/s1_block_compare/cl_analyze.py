import json,sys,numpy as np,collections
R=[json.loads(l) for l in open(sys.argv[1] if len(sys.argv)>1 else 'cl_results.jsonl')]
def metrics(r):
    S=r['states'];x=np.array([s['x'] for s in S]);y=np.array([s['y'] for s in S]);c=np.degrees([s['cmd'] for s in S]);yaw=np.degrees(np.unwrap([s['yaw'] for s in S]))
    e=(x>=120)&(y>-20);m=(x>=110)&(x<122)&(y>-26)
    ce=c[e];P=r['plans'];pe=[p for p in P if p['x']>=120 and p['y']>-20]
    osc=bool(len(ce)) and (np.mean(abs(ce)>=17.0)*100>5 or ce.std()>9)
    return dict(outcome=r['outcome'],std=ce.std() if len(ce) else np.nan,sat=np.mean(abs(ce)>=17.0)*100 if len(ce) else np.nan,
        tv=np.sum(abs(np.diff(ce)))/max(len(ce)*0.1,1e-9) if len(ce)>1 else np.nan,left=c[m].max() if m.any() else np.nan,
        Bmin=min([p['B'] for p in pe]) if pe else np.nan,inv=sum(not p['valid'] for p in P),
        ymax=y[(x>=120)&(x<=135)].max() if ((x>=120)&(x<=135)).any() else np.nan,osc=osc)
G=collections.defaultdict(list)
for r in R: G[(r['config']['name'],r['plant']['name'])].append(metrics(r))
V=list(dict.fromkeys(r['config']['name'] for r in R))
for plant in ['fit_tau0.74_d0.1','tau0.9_d0']:
    print(f"\n== plant {plant} (25 runs each = 5 starts x 5 phases)")
    print(f"{'variant':24s} wrong_br  osc_runs  steer_std  sat%   TV(deg/s)  max_left_mouth  B<3_runs  upper_fail  y_max@x120-135")
    for v in V:
        g=G[(v,plant)];ok=[z for z in g if z['outcome']=='east_done']
        f=lambda k:np.nanmean([z[k] for z in ok])
        print(f"{v:24s} {sum(z['outcome']=='north_branch' for z in g):5d}  {sum(z['osc'] for z in ok):6d}/{len(ok):2d}  {f('std'):8.2f}  {f('sat'):5.1f}  {f('tv'):8.1f}  {f('left'):10.1f}  {sum(z['Bmin']<3 for z in ok):6d}  {sum(z['inv'] for z in g):6d}   {f('ymax'):6.2f}")
