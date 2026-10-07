"""Independent LP feasibility of the production cumulative-increment constraints.
Uses the C++ extracted geometry, not an alternate map interpretation. No vehicle simulation.
"""
from pathlib import Path
import numpy as np,json,itertools
from scipy.optimize import linprog
here=Path(__file__).resolve().parent

def constraints(r,c,chosen):
 ref=np.array(r['reference']);N=len(ref)-1;v=6.;dt=.75;phi=np.tan(.3)*dt/2.8
 seg=np.diff(ref,axis=0);head=np.unwrap(np.arctan2(seg[:,1],seg[:,0]));head=np.r_[head,head[-1]]
 G=np.zeros((N+1,2,2*N))
 for k in range(1,N+1):
  for j in range(k):
   for s in range(j,k):
    G[k,:,2*j]+=dt*np.array([np.cos(head[s]),np.sin(head[s])])
    G[k,:,2*j+1]+=dt*v*np.array([-np.sin(head[s]),np.cos(head[s])])
 nominal=np.zeros(2*N);nominal[1::2]=(np.diff(np.r_[c['yaw'],head[:N]])+np.pi)%(2*np.pi)-np.pi
 A=[];b=[]
 for k,i in enumerate(chosen,1):
  q=np.array(r['cands'][k][i]).reshape(2,2);axis=np.array([-np.sin(head[k]),np.cos(head[k])]);bound=np.sort(q@axis)-ref[k]@axis;g=axis@G[k]
  A.extend([g,-g]);b.extend([bound[1],-bound[0]])
 for k in range(N):
  for sgn in [1,-1]:
   g=np.zeros(2*N);g[2*k]=sgn;A.append(g);b.append(.75-g@nominal)
  for sgn in [1,-1]:
   g=np.zeros(2*N);g[:2*k+1:2]=-phi;g[2*k+1]=sgn;A.append(g);b.append(phi*v-g@nominal)
 return np.array(A),np.array(b)

def run(r,c):
 out=[]
 for sel in itertools.product(*(range(n) for n in r['counts'][1:])):
  A,b=constraints(r,c,sel);z=linprog(np.zeros(A.shape[1]),A_ub=A,b_ub=b,bounds=[(None,None)]*A.shape[1],method='highs')
  out.append(dict(selected=list(sel),feasible=bool(z.success),status=z.status))
 return out
if __name__=='__main__':
 D=json.loads((here/'diagnostic_050627_recorded.json').read_text());C=json.loads((here/'replay_050627.json').read_text())['cycles'];out=[]
 from datetime import datetime
 from zoneinfo import ZoneInfo
 for i,c in enumerate(C):
  ts=datetime.fromtimestamp(c['t'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
  if not '05:06:49'<=ts<='05:06:52.999':continue
  for name in ['cap0_ref0_B0','cap12_ref0_B0','cap12_ref0_B1','cap12_ref1_B0']:
   r=D[name][i];tests=run(r,c);observed=r['selected'][1:]
   assert not r['valid'] or any(z['selected']==observed and z['feasible'] for z in tests),(ts,name,'selected plan LP mismatch')
   out.append(dict(time=ts,name=name,selected=r['selected'],turn_steps=r['turn_steps'],cands=r['cands'],intervals=r['intervals'],combinations=tests))
 (here/'corridor_feasibility.json').write_text(json.dumps(out,indent=2))
 for q in out:
  if q['time'].startswith('05:06:51.906'):print(q['time'],q['name'],q['selected'],q['combinations'])
