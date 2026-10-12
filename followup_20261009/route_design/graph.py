import sys,pickle,numpy as np
sys.path.insert(0,'.')
from roads import cum
rd=pickle.load(open('roads.pkl','rb'))
def seg_proj(c,p):
    a=c[:-1,:2];b=c[1:,:2];v=b-a;L2=np.maximum((v*v).sum(1),1e-12);u=np.clip(((p-a)*v).sum(1)/L2,0,1);q=a+u[:,None]*v;d=np.linalg.norm(q-p,axis=1);i=np.argmin(d)
    s=cum(c);return d[i],s[i]+u[i]*np.sqrt(L2[i]),q[i]
# endpoints
E=[]
for k,r in enumerate(rd):
    c=r['raw'][:,:2];E.append((k,0,c[0]));E.append((k,1,c[-1]))
# cluster endpoints within 7 m
nodes=[];lab={}
for k,e,p in E:
    for ni,n in enumerate(nodes):
        if np.linalg.norm(n['p']-p)<7: n['m'].append((k,e));n['pts'].append(p);n['p']=np.mean(n['pts'],0);lab[(k,e)]=ni;break
    else: nodes.append(dict(p=p.copy(),pts=[p],m=[(k,e)]));lab[(k,e)]=len(nodes)-1
# T-junctions: endpoint near interior of another road
T=[]
for k,e,p in E:
    for j,r in enumerate(rd):
        if j==k: continue
        c=r['raw'];d,s,q=seg_proj(c,p);L=cum(c)[-1]
        if d<r['base']/2+2.5 and 4<s<L-4: T.append((k,e,j,s,d))
if __name__=="__main__": print('nodes',len(nodes))
for i,n in (enumerate(nodes) if __name__=="__main__" else []):
    print(i,np.round(n['p'],1),[(rd[k]['id'],'S' if e==0 else 'E') for k,e in n['m']])
if __name__=="__main__": print('T-junctions:');[print(' road',rd[k]['id'],'S' if e==0 else 'E','-> on road',rd[j]['id'],'at s=%.1f d=%.1f'%(s,d)) for k,e,j,s,d in T]
pickle.dump(dict(nodes=nodes,lab=lab,T=T),open('graph_raw.pkl','wb'))
