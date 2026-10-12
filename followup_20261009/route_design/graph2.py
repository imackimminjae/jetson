import sys,pickle,numpy as np,collections
sys.path.insert(0,'.')
from roads import cum
from graph import seg_proj
rd=pickle.load(open('roads.pkl','rb'));G=pickle.load(open('graph_raw.pkl','rb'));nodes,lab,T=G['nodes'],G['lab'],G['T']
splits=collections.defaultdict(list)
for k,r in enumerate(rd):
    L=cum(r['raw'])[-1];splits[k]+= [(0.0,lab[(k,0)]),(L,lab[(k,1)])]
seen=set()
for k,e,j,s,d in T:
    n=lab[(k,e)]
    if (j,n) in seen: continue
    seen.add((j,n));splits[j].append((s,n))
edges=[]
for j,sp in splits.items():
    sp=sorted(sp)
    for (s0,n0),(s1,n1) in zip(sp[:-1],sp[1:]):
        if s1-s0<0.5: continue
        if n0==n1 and s1-s0<5: continue
        edges.append(dict(i=len(edges),road=j,rid=rd[j]['id'],s0=s0,s1=s1,n0=n0,n1=n1,L=s1-s0))
def piece(e,forward=True):
    c=rd[e['road']]['raw'][:,:2];s=cum(rd[e['road']]['raw'])
    ss=np.arange(e['s0'],e['s1'],0.5);ss=np.append(ss,e['s1'])
    P=np.c_[np.interp(ss,s,c[:,0]),np.interp(ss,s,c[:,1])]
    return P if forward else P[::-1]
if __name__=="__main__": print("edges",len(edges))
if __name__=="__main__":
  for e in edges: print(f"e{e['i']:2d} road {str(e['rid']):5s} n{e['n0']:2d}->n{e['n1']:2d} L={e['L']:6.1f}")
pickle.dump(dict(edges=edges,nodes=nodes),open('graph.pkl','wb'))
