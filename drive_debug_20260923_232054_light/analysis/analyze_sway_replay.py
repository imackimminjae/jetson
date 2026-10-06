import pickle,json,time
import numpy as np
from pathlib import Path
from scipy.optimize import minimize,LinearConstraint,Bounds
from datetime import datetime
root=Path('/home/imac/ros2_ws/drive_debug_20260923_232054_light/analysis');d=pickle.load((root/'decoded.pkl').open('rb'));l=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]);paths=d['/planner/lower_reference_path/with_arclength']
wrap=lambda a:np.arctan2(np.sin(a),np.cos(a))
for p in paths:
 p['psi']=np.unwrap(np.arctan2(np.diff(p['xy'],axis=0)[:,1],np.diff(p['xy'],axis=0)[:,0]));p['psi']=np.r_[p['psi'],p['psi'][-1]]
 p['kap']=np.array([(p['psi'][min(i+1,len(p['s'])-1)]-p['psi'][max(i-1,0)])/(p['s'][min(i+1,len(p['s'])-1)]-p['s'][max(i-1,0)]) for i in range(len(p['s']))])
def ref(x,p):
 j=np.argmin(np.sum((p['xy']-x[1:3])**2,axis=1));psi=p['psi'][j];ey=np.dot([-np.sin(psi),np.cos(psi)],x[1:3]-p['xy'][j]);eps=wrap(x[3]-psi);q=p['s'][j]+np.arange(30)*max(0,x[4])*.1;k=np.interp(q,p['s'],p['kap']);return j,ey,eps,k
pt=np.array([p['t'] for p in paths]);indices=[];mismatch=[]
for x in l:
 approx=np.searchsorted(pt,x[0]+.003,side='right')-1;candidates=range(max(0,approx-2),min(len(paths),approx+2));scores=[]
 for k in candidates:
  j,ey,eps,cur=ref(x,paths[k]);scores.append(abs(ey-x[6])+abs(wrap(eps-x[7]))+abs(cur[0]-x[28])+abs(j-x[5]))
 best=np.argmin(scores);indices.append(list(candidates)[best]);mismatch.append(scores[best])
indices=np.array(indices);print('REFERENCE replay max mismatch',max(mismatch),'bad',sum(x>1e-6 for x in mismatch),flush=True)
N=30;diff=np.eye(N)-np.eye(N,k=-1);Q=np.tile([1,.5],N);constraints=np.vstack([np.eye(N),diff]);g=1-np.exp(-.1);phi=1-g

def solve(x,p,history):
 j,ey,eps,kappa=ref(x,p);v=x[4];sys=np.array([[1,v*.1,0],[0,1,v/2.8*g],[0,0,phi]]);state=np.array([ey,eps,x[19]]);imap=np.zeros((3,N));a=[];b=[]
 for k in range(N):
  ci=k-3;known=history[k] if ci<0 else 0
  state=sys@state+np.array([0,v/2.8*(.1-g)*known-v*.1*kappa[k],g*known]);imap=sys@imap
  if ci>=0:imap[1,ci]+=v/2.8*(.1-g);imap[2,ci]+=g
  a.extend(state[:2]);b.extend(imap[:2])
 a=np.array(a);b=np.array(b);H=2*(b.T@(Q[:,None]*b)+.3*np.eye(N)+8*diff.T@diff)+1e-9*np.eye(N);f=2*b.T@(Q*a);f[0]-=16*x[29]
 low=np.r_[np.full(N,-.3),np.full(N,-np.pi/30)];high=-low;low[N]+=x[29];high[N]+=x[29]
 result=minimize(lambda u:.5*u@H@u+f@u,np.full(N,x[29]),jac=lambda u:H@u+f,method='SLSQP',constraints=[LinearConstraint(constraints,low,high)],options={'ftol':1e-10,'maxiter':120})
 if not result.success:raise RuntimeError(result.message)
 return result.x[0]/.3
rows=[];errors=[]
for i in range(3,len(l)):
 x=l[i];pid=indices[i];hist=l[i-3:i,9]*.3
 if not (1790173287<x[0]<1790173306):continue
 actual=solve(x,paths[pid],hist);errors.append(abs(actual-x[9]));refresh=pid!=indices[i-1]
 previous=solve(x,paths[indices[i-1]],hist) if refresh else actual
 # Same pose projected onto old/current reference isolates path update.
 oldref=ref(x,paths[indices[i-1]]);newref=ref(x,paths[pid]);rows.append(dict(t=x[0],path=int(pid),refresh=bool(refresh),measured=x[9],replay=actual,old_path_command=previous,path_command_change=actual-previous,heading_jump_deg=float(np.degrees(wrap(oldref[2]-newref[2]))),ey_change=newref[1]-oldref[1],ey=x[6],eps_deg=np.degrees(x[7]),nearest=int(x[5]),curvature=x[28]))
print('QP replay median/max error normalized',np.median(errors),max(errors),flush=True)
for x in sorted([r for r in rows if r['refresh']],key=lambda r:abs(r['path_command_change']),reverse=True)[:12]:print('REFRESH',datetime.fromtimestamp(x['t']).strftime('%H:%M:%S.%f')[:-3],json.dumps(x),flush=True)
# Abrupt recorded command changes with and without path refresh.
changes=np.diff(l[:,9]);new=indices[1:]!=indices[:-1]
print('COMMAND changes >.3',int(sum(abs(changes)>.3)),'onrefresh',int(sum((abs(changes)>.3)&new)), 'total variation refresh/other',sum(abs(changes[new])),sum(abs(changes[~new])))
json.dump(rows,(root/'sway_replay.json').open('w'),indent=2)
np.savez(root/'sway_reference_match.npz',indices=indices,mismatch=mismatch)
