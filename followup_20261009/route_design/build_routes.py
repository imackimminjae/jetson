import sys,pickle,json,math,numpy as np
sys.path.insert(0,'.')
from scipy.ndimage import gaussian_filter1d
import gen
from gen import OUT,DEAD,turn,din
S=pickle.load(open('chosen.pkl','rb'));chosen=S['chosen']
def extend(trail):
    trail=list(trail);used={h['e'] for h in trail};extra=0
    while trail[-1]['b'] not in DEAD and trail[-1]['L']<30 and extra<60:
        seen={trail[0]['a']}|{x['b'] for x in trail};opts=[h for h in OUT[trail[-1]['b']] if h['e'] not in used and h['b'] not in seen and turn(trail[-1],h)<=60]
        if not opts: break
        h=min(opts,key=lambda h:turn(trail[-1],h));trail.append(h);used.add(h['e']);extra+=h['L']
    return trail
def resample(P,step=0.5):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);keep=np.r_[True,np.diff(s)>1e-6];P,s=P[keep],s[keep]
    t=np.arange(0,s[-1],step);t=np.append(t,s[-1]);return np.c_[np.interp(t,s,P[:,0]),np.interp(t,s,P[:,1])],t
def dp(P,tol):
    if len(P)<3: return [0,len(P)-1]
    a,b=P[0],P[-1];v=b-a;n=np.linalg.norm(v);d=np.abs(np.cross(v,P-a))/n if n>1e-9 else np.linalg.norm(P-a,axis=1);i=int(np.argmax(d))
    if d[i]<=tol: return [0,len(P)-1]
    l=dp(P[:i+1],tol);r=dp(P[i:],tol);return l[:-1]+[x+i for x in r]
def chord_ok(a,b,m=0.6):
    n=max(2,int(np.linalg.norm(b-a)/0.5));Q=a+np.outer(np.linspace(0,1,n),b-a);return din(Q).min()>=m
routes=[]
for k,trail in enumerate(chosen):
    trail=extend(trail)
    P=np.vstack([h['P'] for h in trail]);P,s=resample(P)
    # trim: dead-end start 6 m, long-edge start leaves <=50 m before first junction; end 10 m before dead end or 8 m before junction
    L0=trail[0]['L'];start=6.0 if trail[0]['a'] in DEAD else max(6.0,L0-50.0)
    end=s[-1]-(10.0 if trail[-1]['b'] in DEAD else 8.0)
    m=(s>=start)&(s<=end);P=P[m]
    for sig in (6,4,3,2,1.2,0):   # smoothing sigma in samples*0.5m: up to 3 m
        Q=P.copy()
        if sig>0: Q[1:-1]=np.c_[gaussian_filter1d(P[:,0],sig*2,mode='nearest'),gaussian_filter1d(P[:,1],sig*2,mode='nearest')][1:-1]
        if din(Q).min()>=1.3: break
    Q,_=resample(Q)
    idx=dp(Q,0.6)
    # enforce spacing: split long chords (>30 m), check chords inside road, then merge short ones (<10 m)
    wp=[Q[i] for i in idx];out=[wp[0]]
    for a,b in zip(wp[:-1],wp[1:]):
        n=int(np.ceil(np.linalg.norm(b-a)/30.0))
        for j in range(1,n+1): out.append(a+(b-a)*j/n)
    wp=out;changed=True
    while changed:
        changed=False
        for i in range(1,len(wp)-1):
            if min(np.linalg.norm(wp[i]-wp[i-1]),np.linalg.norm(wp[i+1]-wp[i]))<10 and chord_ok(wp[i-1],wp[i+1]) and np.linalg.norm(wp[i+1]-wp[i-1])<=32:
                del wp[i];changed=True;break
    wp=np.array(wp);bad=[i for i in range(len(wp)-1) if not chord_ok(wp[i],wp[i+1],0.3)]
    sp=np.linalg.norm(np.diff(wp,axis=0),axis=1);hd=np.degrees(np.arctan2(*(np.diff(wp,axis=0).T[::-1])));tw=np.abs((np.diff(hd)+180)%360-180)
    routes.append(dict(k=k,trail=[(h['e'],h['a'],h['b']) for h in trail],dense=Q,wp=wp,len=float(np.sum(np.linalg.norm(np.diff(Q,axis=0),axis=1))),
        din_min=float(din(Q).min()),bad_chords=bad,spacing=(float(sp.min()),float(sp.max())),max_wp_turn=float(tw.max()) if len(tw) else 0.0,
        start_heading=float(math.atan2(*(Q[min(8,len(Q)-1)]-Q[0])[::-1])),start_dead=trail[0]['a'] in DEAD,end_dead=trail[-1]['b'] in DEAD,nodes=[trail[0]['a']]+[h['b'] for h in trail]))
    r=routes[-1];print(f"R{k+1:2d} len {r['len']:5.0f} m wp {len(wp):2d} spacing {r['spacing'][0]:4.1f}-{r['spacing'][1]:4.1f} min road clearance {r['din_min']:.2f} m bad chords {bad} max wp turn {r['max_wp_turn']:.0f} nodes {r['nodes']}")
pickle.dump(routes,open('routes.pkl','wb'))
