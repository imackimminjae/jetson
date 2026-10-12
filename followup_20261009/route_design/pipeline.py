# End-to-end: candidate trails (turn limit + bans) -> greedy cover -> extend ends -> trim -> racing-line optimize -> feasibility loop.
import sys,pickle,json,csv,math,numpy as np,collections
sys.path.insert(0,'.')
from gen import H,OUT,DEAD,turn,USE
from graph2 import piece
from band import optimize,radius,sample,DIN
from routesel import covered_by  # reuses map-matching helper
D='/home/imac/ros2_ws/src/virtual_control/data/knu_routes/'
MAXTURN=95;MAXLEN=350
def candidates(bans):
    cands=[]
    def dfs(trail,length,used,visited):
        last=trail[-1];dead=last['b'] in DEAD
        if length>=110 or dead: cands.append(list(trail))
        if dead: return
        for h in OUT[last['b']]:
            if length+h['L']>MAXLEN+(30 if h['b'] in DEAD else 0): continue
            if h['e'] in used or h['b'] in visited: continue
            if turn(last,h)>MAXTURN or (last['e'],last['b'],h['e']) in bans: continue
            trail.append(h);used.add(h['e']);visited.add(h['b']);dfs(trail,length+h['L'],used,visited);trail.pop();used.discard(h['e']);visited.discard(h['b'])
    for h in H:
        if h['a'] in DEAD or h['L']>=60: dfs([h],h['L'],{h['e']},{h['a'],h['b']})
    return cands
pre=set()
for n in (1,2,3):
    P=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(D+f'scenario{n}.csv'))])
    if n==2: P=np.vstack([P[P[:,0]<-76],[-75,67]])
    pre|=covered_by(P,tol=8.0 if n==2 else 5.0)
W={e['i']:max(e['L'],12.0) for e in USE}
def cover(cands):
    unc=set(W)-pre;chosen=[]
    while unc:
        best=None
        for c in cands:
            es={h['e'] for h in c};gain=sum(W[i] for i in es&unc)
            if gain<=0: continue
            score=gain-0.25*sum(W[i] for i in es-unc)+(15 if c[-1]['b'] in DEAD else 0)
            if best is None or score>best[0]: best=(score,c)
        if best is None: break
        chosen.append(best[1]);unc-={h['e'] for h in best[1]}
    return chosen,unc
def extend(trail,bans):
    trail=list(trail);used={h['e'] for h in trail};extra=0
    while trail[-1]['b'] not in DEAD and trail[-1]['L']<30 and extra<60:
        seen={trail[0]['a']}|{x['b'] for x in trail}
        opts=[h for h in OUT[trail[-1]['b']] if h['e'] not in used and h['b'] not in seen and turn(trail[-1],h)<=60 and (trail[-1]['e'],trail[-1]['b'],h['e']) not in bans]
        if not opts: break
        h=min(opts,key=lambda h:turn(trail[-1],h));trail.append(h);used.add(h['e']);extra+=h['L']
    return trail
def rs(P,step=0.5):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);k=np.r_[True,np.diff(s)>1e-6];P,s=P[k],s[k];t=np.arange(0,s[-1],step);t=np.append(t,s[-1])
    return np.c_[np.interp(t,s,P[:,0]),np.interp(t,s,P[:,1])],t
def geometry(trail):
    P,s=rs(np.vstack([h['P'] for h in trail]))
    start=6.0 if trail[0]['a'] in DEAD else max(6.0,trail[0]['L']-50.0);end=s[-1]-(10.0 if trail[-1]['b'] in DEAD else 8.0)
    # node arc positions to attribute problems to transitions
    acc=0;nodes_s=[]
    for h in trail[:-1]: acc+=np.sum(np.linalg.norm(np.diff(h['P'],axis=0),axis=1));nodes_s.append(acc)
    m=(s>=start)&(s<=end);Z,rm,cl=optimize(P[m],clear=1.15,mu=2000)
    return Z,rm,cl,start,nodes_s
bans=set();routes=None
for loop in range(12):
    cands=candidates(bans);chosen,unc=cover(cands)
    chosen=[extend(t,bans) for t in chosen];bad=0;routes=[]
    for t in chosen:
        Z,rm,cl,start,ns=geometry(t)
        routes.append(dict(trail=t,dense=Z,rmin=rm,clear=cl))
        if rm<9.5 or cl<0.9:
            bad+=1;rad=radius(Z);i=int(np.argmin(rad))+10 if rm<9.5 else int(np.argmin(sample(DIN,Z)))
            # map back: arc position along original trail ~ start + arc on Z
            sz=start+np.sum(np.linalg.norm(np.diff(Z[:i+1],axis=0),axis=1));j=int(np.argmin([abs(x-sz) for x in ns])) if ns else None
            if j is not None: bans.add((t[j]['e'],t[j]['b'],t[j+1]['e']))
    print(f"loop {loop}: routes {len(routes)} infeasible {bad} uncoverable edges {sorted(unc)} bans {len(bans)}",flush=True)
    if bad==0: break
pickle.dump(dict(routes=routes,bans=bans,unc=unc),open('pipeline_out.pkl','wb'))
for k,r in enumerate(routes):
    t=r['trail'];print(f"R{k+1:2d} len {np.sum(np.linalg.norm(np.diff(r['dense'],axis=0),axis=1)):5.0f} m rmin {r['rmin']:5.1f} clear {r['clear']:.2f} nodes {[t[0]['a']]+[h['b'] for h in t]}")
