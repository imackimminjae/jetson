import sys,pickle,numpy as np,itertools
sys.path.insert(0,'.')
from gen import H,OUT,turn
from band import optimize,radius,sample,DIN
def tail(P,L):  # last L metres
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);return P[s>=s[-1]-L]
def head(P,L):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);return P[s<=L]
res={}
for node,outs in OUT.items():
    ins=[h for h in H if h['b']==node]
    for hi in ins:
        for ho in outs:
            if ho['e']==hi['e']: continue
            t=turn(hi,ho)
            if t>135: continue
            P=np.vstack([tail(hi['P'],25),head(ho['P'],25)])
            if t<20: res[(hi['e'],hi['a'],hi['b'],ho['e'],ho['b'])]=dict(turn=t,rmin=99.0,clear=float(sample(DIN,P).min()),ok=True);continue
            Z,rm,cl=optimize(P,clear=1.1,mu=2000)
            res[(hi['e'],hi['a'],hi['b'],ho['e'],ho['b'])]=dict(turn=t,rmin=rm,clear=cl,ok=rm>=10.0 and cl>=0.9)
ok=sum(v['ok'] for v in res.values());print('transitions',len(res),'feasible',ok)
for k,v in sorted(res.items(),key=lambda kv:kv[1]['rmin']):
    if not v['ok']: print(' infeasible: in e%d n%d->n%d out e%d ->n%d turn %.0f rmin %.1f clear %.2f'%(k[0],k[1],k[2],k[3],k[4],v['turn'],v['rmin'],v['clear']))
pickle.dump(res,open('transitions.pkl','wb'))
