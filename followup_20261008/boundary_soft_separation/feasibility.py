"""Independent feasibility of the recorded-state upper linear constraints, not a vehicle simulator."""
from pathlib import Path
import numpy as np,json,gzip,itertools,copy,yaml
from scipy.optimize import linprog
H=Path(__file__).resolve().parent

def constraints(r,c,chosen,drop=()):
 ref=np.array(r['reference']);N=len(ref)-1;v=6.;dt=.75;phi=np.tan(.3)*dt/2.8
 head=np.array(r['psirk'])+c['yaw'];G=np.zeros((N+1,2,2*N))
 for k in range(1,N+1):
  for j in range(k):
   for s in range(j,k):
    G[k,:,2*j]+=dt*np.array([np.cos(head[s]),np.sin(head[s])]);G[k,:,2*j+1]+=dt*v*np.array([-np.sin(head[s]),np.cos(head[s])])
 nominal=np.zeros(2*N);nominal[1::2]=(np.diff(np.r_[c['yaw'],head[:N]])+np.pi)%(2*np.pi)-np.pi;A=[];b=[]
 for k,i in enumerate(chosen,1):
  if k in drop or r['side_boundary_tail_start']>=0 and k>=r['side_boundary_tail_start']:continue
  q=np.array(r['cands'][k][i]).reshape(2,2);axis=np.array([-np.sin(head[k]),np.cos(head[k])]);bound=np.sort(q@axis)-ref[k]@axis;g=axis@G[k];A.extend([g,-g]);b.extend([bound[1],-bound[0]])
 for k in range(N):
  for sign in [1,-1]:
   g=np.zeros(2*N);g[2*k]=sign;A.append(g);b.append(.75-g@nominal)
  for sign in [1,-1]:
   g=np.zeros(2*N);g[:2*k+1:2]=-phi;g[2*k+1]=sign;A.append(g);b.append(phi*v-g@nominal)
 return np.array(A),np.array(b)
def run(r,c,drop=()):
 if len(r["reference"])<2:return []
 rows=[]
 for sel in itertools.product(*(range(n) for n in r['counts'][1:])):
  A,b=constraints(r,c,sel,drop);z=linprog(np.zeros(A.shape[1]),A_ub=A,b_ub=b,bounds=[(None,None)]*A.shape[1],method='highs');rows.append(dict(selected=list(sel),feasible=bool(z.success)))
 return rows
out=[]
for tag,inds in [('batch05',[46,47,48,49,50]),('batch07',list(range(53,65)))]:
 cs=json.loads((H/f'input_{tag}.json').read_text())['cycles'];r=json.loads(gzip.decompress((H/f'result_{tag}.json.gz').read_bytes()));q=json.loads(gzip.decompress((H/f'refine_{tag}.json.gz').read_bytes()))
 for j in inds:
  for name in ['base_fixed','clip_raw_fixed','edge_carried','gated_skip_carried']:
   z=(q if name in q else r)[name][j];tests=run(z,cs[j]);assert not z['valid'] or any(t['feasible'] and t['selected']==z['selected'][1:] for t in tests),(tag,j,name)
   drops={str(k):any(t['feasible'] for t in run(z,cs[j],(k,))) for k in range(1,len(z['counts']))} if not z['valid'] else {}
   out.append(dict(tag=tag,index=j,mode=name,valid=z['valid'],horizon=len(z['counts'])-1,combinations=tests,feasible_if_drop_single_stage=drops))
(H/'feasibility.json').write_text(json.dumps(out,indent=2));print('LP cases',len(out),'selected feasible checks pass')
