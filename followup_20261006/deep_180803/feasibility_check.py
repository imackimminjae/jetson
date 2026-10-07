from pathlib import Path
import json,numpy as np,sys,itertools
from scipy.optimize import linprog
H=Path(__file__).resolve().parent;sys.path.insert(0,str(H.parent/'reference_B_separation'))
from feasibility import constraints
C=json.loads((H/'input.json').read_text())['cycles'];D=json.loads((H/'result.json').read_text());out=[]
for i,c in enumerate(C):
 if not 230<c['x']<255:continue
 for name in ['baseline','raw_corridors','B5','fresh_ref','force_turn4','hold_wp5_turn4']:
  r=D[name][i];N=len(r['counts'])-1
  if N<=0 or any(n==0 for n in r['counts'][1:]):continue
  ref=np.array(r['reference']);head=np.array(r['psirk'])+c['yaw'];G=np.zeros((2,2*N))
  for j in range(N):
   for k in range(j,N):G[:,2*j]+=.75*np.array([np.cos(head[k]),np.sin(head[k])]);G[:,2*j+1]+=4.5*np.array([-np.sin(head[k]),np.cos(head[k])])
  tests=[]
  for sel in itertools.product(*(range(n) for n in r['counts'][1:])):
   A,b=constraints(r,c,sel);z=linprog(np.zeros(2*N),A_ub=A,b_ub=b,bounds=[(None,None)]*(2*N),method='highs')
   # terminal in NE arm vicinity; test of linearized model feasibility, not physical driveability
   At=np.array([G[0],-G[0],G[1],-G[1]]);bt=np.array([280-ref[-1,0],ref[-1,0]-255,170-ref[-1,1],ref[-1,1]-145]);zz=linprog(np.zeros(2*N),A_ub=np.r_[A,At],b_ub=np.r_[b,bt],bounds=[(None,None)]*(2*N),method='highs')
   zy=linprog(-G[1],A_ub=np.r_[A,[G[0],-G[0]]],b_ub=np.r_[b,280-ref[-1,0],ref[-1,0]-255],bounds=[(None,None)]*(2*N),method='highs')
   tests.append({'sel':list(sel),'feasible':bool(z.success),'NE_box_feasible':bool(zz.success),'max_terminal_y_in_xrange':float(ref[-1,1]+G[1]@zy.x) if zy.success else None})
  if r['valid']:assert any(v['feasible'] and v['sel']==r['selected'][1:] for v in tests),(i,name)
  out.append({'i':i,'t':c['t'],'xy':[c['x'],c['y']],'variant':name,'combinations':tests})
(H/'feasibility.json').write_text(json.dumps(out,indent=2));print('checked',len(out),'cases')
