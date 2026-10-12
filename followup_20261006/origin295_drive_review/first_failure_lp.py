from pathlib import Path
import numpy as np,json,itertools,sys,copy
from scipy.optimize import linprog
H=Path(__file__).resolve().parent;sys.path.insert(0,str(H.parent/'reference_B_separation'));from feasibility import constraints
out=[]
for tag in ['184905','185025','185147']:
 cs=json.loads((H/f'input_{tag}.json').read_text())['cycles'];ds=json.loads((H/f'replay_{tag}.json').read_text());i=next(i for i,c in enumerate(cs) if not c['rec_valid']);c=cs[i];r=ds['fixed_295'][i];N=len(r['reference'])-1;tests=[]
 for mode in ['full','drop_k1','raw_k1','raw_all']:
  z=copy.deepcopy(r)
  if mode.startswith('raw'):
   for iv in z['intervals']:
    k,j=iv['k'],iv['i']
    if mode=='raw_k1' and k!=1:continue
    if k>=len(z['cands']):continue
    ab=np.array(z['cands'][k][j]).reshape(2,2);u=ab[1]-ab[0];u/=np.linalg.norm(u);m=ab.mean(0);z['cands'][k][j]=np.r_[m-u*iv['raw']/2,m+u*iv['raw']/2].tolist()
  vals=[]
  for sel in itertools.product(*(range(n) for n in z['counts'][1:])):
   A,b=constraints(z,c,sel)
   if mode=='drop_k1':A,b=A[2:],b[2:]
   q=linprog(np.zeros(2*N),A_ub=A,b_ub=b,bounds=[(None,None)]*(2*N),method='highs');prefix=[]
   if mode!='drop_k1':
    for k in range(1,N+1):
     inds=list(range(2*k))+list(range(2*N,len(b)));qq=linprog(np.zeros(2*N),A_ub=A[inds],b_ub=b[inds],bounds=[(None,None)]*(2*N),method='highs');prefix.append(bool(qq.success))
   vals.append({'sel':sel,'feasible':bool(q.success),'prefix_feasible':prefix})
  tests.append({'mode':mode,'feasible_any':any(v['feasible'] for v in vals),'combinations':vals})
 out.append({'tag':tag,'i':i,'t':c['t'],'N':N,'tests':tests});print(tag,tests,flush=True)
(H/'first_failure_lp.json').write_text(json.dumps(out,indent=2))
