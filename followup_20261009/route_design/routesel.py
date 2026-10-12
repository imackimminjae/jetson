import sys,pickle,json,csv,math,numpy as np,collections
sys.path.insert(0,'.')
from graph2 import piece
C=pickle.load(open('cands.pkl','rb'));cands,USE=C['cands'],C['USE']
D='/home/imac/ros2_ws/src/virtual_control/data/knu_routes/'
def dense(P):
    out=[P[0]]
    for a,b in zip(P[:-1],P[1:]):
        n=max(1,int(np.linalg.norm(b-a)/0.5));out+=[a+(b-a)*t for t in np.linspace(0,1,n+1)[1:]]
    return np.array(out)
def covered_by(P,tol=5.0,frac=.8):
    Pd=dense(P);cov=set()
    for e in USE:
        S=piece(e)[::4];d=np.min(np.linalg.norm(S[:,None,:]-Pd[None,:,:],axis=2),axis=1)
        if np.mean(d<tol)>=frac: cov.add(e['i'])
    return cov
if __name__=="__main__":
  pre=set()  
  if __name__!="__main__": pass
  for n in (1,2,3):
      P=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(D+f'scenario{n}.csv'))])
      if n==2: P=np.vstack([P[P[:,0]<-76],[-75,67]])
      c=covered_by(P,tol=8.0 if n==2 else 5.0);print('scenario',n,'covers edges',sorted(c));pre|=c
  W={e['i']:max(e['L'],12.0) for e in USE}
  unc=set(W)-pre;print('uncovered at start',len(unc),'of',len(W),'length %.0f m'%sum(W[i] for i in unc))
  chosen=[]
  while unc:
      best=None
      for c in cands:
          es={h['e'] for h in c};gain=sum(W[i] for i in es&unc)
          if gain<=0: continue
          L=sum(h['L'] for h in c);tn=len(c)
          score=gain-0.25*sum(W[i] for i in es-unc)+(15 if c[-1]['b'] in C['DEAD'] else 0)   # mild penalty for re-driving covered roads
          if best is None or score>best[0]: best=(score,c,gain,L)
      if best is None: print('cannot cover',unc);break
      _,c,gain,L=best;chosen.append(c);unc-={h['e'] for h in c}
      print(f"route {len(chosen)}: len {L:.0f} m, new {gain:.0f}, edges {[h['e'] for h in c]} nodes {[c[0]['a']]+[h['b'] for h in c]}")
  pickle.dump(dict(chosen=chosen,pre=pre),open('chosen.pkl','wb'))
  