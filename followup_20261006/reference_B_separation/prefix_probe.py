from feasibility import constraints,here
import numpy as np,json,copy
from scipy.optimize import linprog
D=json.loads((here/'diagnostic_050627_recorded.json').read_text());C=json.loads((here/'replay_050627.json').read_text())['cycles'];i=min(range(len(C)),key=lambda j:abs(C[j]['t']-1791230811.906))
# Identify by stored critical time rather than assuming timestamp conversion.
from datetime import datetime
from zoneinfo import ZoneInfo
i=next(j for j,c in enumerate(C) if datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f').startswith('05:06:51.906'))
def feasible(A,b):return bool(linprog(np.zeros(A.shape[1]),A_ub=A,b_ub=b,bounds=[(None,None)]*A.shape[1],method='highs').success)
out={}
for name in ['cap0_ref0_B0','cap12_ref0_B0','cap12_ref0_B1']:
 r=D[name][i];N=len(r['counts'])-1;sel=[0]*N;A,b=constraints(r,C[i],sel)
 prefix={str(k):feasible(np.r_[A[:2*k],A[2*N:]],np.r_[b[:2*k],b[2*N:]]) for k in range(1,N+1)}
 remove={str(k):feasible(np.delete(A,[2*k-2,2*k-1],axis=0),np.delete(b,[2*k-2,2*k-1])) for k in range(1,N+1)}
 raw=copy.deepcopy(r)
 for z in r['intervals']:
  k,j=z['k'],z['i']
  if k>=len(raw['cands']) or j>=len(raw['cands'][k]):continue
  q=np.array(raw['cands'][k][j]).reshape(2,2);axis=(q[1]-q[0])/np.linalg.norm(q[1]-q[0]);mid=q.mean(axis=0);raw['cands'][k][j]=np.r_[mid-z['raw']/2*axis,mid+z['raw']/2*axis].tolist()
 Ar,br=constraints(raw,C[i],sel)
 # minimum uniform lane-bound relaxation in metres, inputs remain exact.
 aug=np.c_[A,np.r_[-np.ones(2*N),np.zeros(len(A)-2*N)]]
 lp=linprog(np.r_[np.zeros(A.shape[1]),1.],A_ub=aug,b_ub=b,bounds=[(None,None)]*A.shape[1]+[(0,None)],method='highs')
 out[name]=dict(east_all_zero_prefix_feasible=prefix,remove_single_stage_feasible=remove,raw_all_stages_feasible=feasible(Ar,br),uniform_min_relax_m=float(lp.x[-1]) if lp.success else None,k5_k6_k7={str(k):r['cands'][k] for k in [5,6,7]})
(here/'prefix_probe.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
