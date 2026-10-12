from pathlib import Path
import json,sys,itertools,numpy as np
from scipy.optimize import linprog
from datetime import datetime
from zoneinfo import ZoneInfo
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'reference_B_separation'))
from feasibility import constraints
out=[]
for tag in ['184905','185025','185147']:
 cs=json.loads((H.parent/'origin295_drive_review'/f'input_{tag}.json').read_text())['cycles']
 ds=json.loads((H/f'result_{tag}.json').read_text())
 i=next(i for i,c in enumerate(cs) if not c['rec_valid']);c=cs[i]
 for name in ['carried_m1','carried_m2']:
  r=ds[name][i];N=len(r['reference'])-1;comb=[]
  for sel in itertools.product(*(range(n) for n in r['counts'][1:])):
   A,b=constraints(r,c,sel);prefix=[];singles=[]
   for k in range(2,N+1):
    inds=list(range(2,2*k))+list(range(2*N,len(b)))
    z=linprog(np.zeros(2*N),A_ub=A[inds],b_ub=b[inds],bounds=[(None,None)]*(2*N),method='highs')
    prefix.append(bool(z.success))
    inds=list(range(2*k-2,2*k))+list(range(2*N,len(b)))
    z=linprog(np.zeros(2*N),A_ub=A[inds],b_ub=b[inds],bounds=[(None,None)]*(2*N),method='highs')
    singles.append(bool(z.success))
   comb.append(dict(selected=sel,k2_onward_prefix_feasible=prefix,each_k2_onward_alone_feasible=singles))
  row=dict(tag=tag,i=i,time=datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).isoformat(),variant=name,N=N,combinations=comb)
  out.append(row);print(row)
(H/'first_failure_lp.json').write_text(json.dumps(out,indent=2))
