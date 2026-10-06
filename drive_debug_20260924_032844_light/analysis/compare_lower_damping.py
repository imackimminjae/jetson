from pathlib import Path
import pickle,json,numpy as np
from scipy.optimize import minimize,LinearConstraint
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'));tr=np.array([e['data'] for e in d['/debug/lower_mpc_trace']]);paths=d['/planner/lower_reference_path/with_arclength'];pt=np.array([e['t'] for e in paths]);N=30;dt=.1;L=2.8;ratio=.5;tau=.05;decay=np.exp(-dt/tau);Is=tau*(1-decay);Ic=dt-Is;Js=tau*Ic;Jc=.5*dt*dt-Js
C=np.array([[1.,0,0],[0,1,ratio]]);Q=np.diag(np.tile([1,.5],N));D=np.eye(N)-np.eye(N,k=-1);rate=1.0471975512*dt
wrap=lambda x:np.arctan2(np.sin(x),np.cos(x))
prepared=[]
for row in tr:
 if row[10]!=1:prepared.append(None);continue
 p=paths[max(0,np.searchsorted(pt,row[0],side='right')-1)];xy=p['xy'];s=p['s'];delta=np.diff(xy,axis=0);heading=np.unwrap(np.arctan2(delta[:,1],delta[:,0]));heading=np.r_[heading,heading[-1]];nearest=np.argmin(np.linalg.norm(xy-row[1:3],axis=1));psi=heading[nearest];normal=np.array([-np.sin(psi),np.cos(psi)]);ey=normal@(row[1:3]-xy[nearest]);ep=wrap(row[3]-psi);v=row[4]
 A=np.array([[1,v*dt,v*ratio*Is+v*v/L*Js],[0,1,v/L*Is],[0,0,decay]]);B=np.array([v*ratio*Ic+v*v/L*Jc,v/L*Ic,1-decay]);F=np.zeros((2*N,3));G=np.zeros((2*N,N));ak=np.eye(3);g=np.zeros((3,N))
 for k in range(N):
  ak=A@ak;g=A@g;g[:,k]+=B;F[2*k:2*k+2]=C@ak;G[2*k:2*k+2]=C@g
 query=np.minimum(s[-1],s[nearest]+np.arange(1,N+1)*v*dt);future=np.c_[np.interp(query,s,xy[:,0]),np.interp(query,s,xy[:,1])];target=np.zeros(2*N);target[::2]=(future-xy[nearest])@normal;target[1::2]=wrap(np.interp(query,s,heading)-psi)
 prepared.append((F,G,target,np.array([ey,ep]),nearest))
outputs={}
for rd in [8.,20.,40.,80.]:
 eff=0.;prev=0.;guess=np.zeros(N);out=[];check=[];fails=0
 for row,p in zip(tr,prepared):
  if p is None:eff=prev=0.;guess=np.zeros(N);out.append(0.);continue
  F,G,target,err,nearest=p
  state=np.r_[err,eff];H=2*(G.T@Q@G+.3*np.eye(N)+rd*D.T@D);H+=np.eye(N)*1e-9;f=2*G.T@Q@(F@state-target);f[0]-=2*rd*prev
  lo=np.full(N,-rate);hi=np.full(N,rate);lo[0]+=prev;hi[0]+=prev
  result=minimize(lambda u:.5*u@H@u+f@u,guess,jac=lambda u:H@u+f,method='SLSQP',bounds=[(-.3,.3)]*N,constraints=[LinearConstraint(D,lo,hi)],options={'ftol':1e-10,'maxiter':100})
  if not result.success:fails+=1
  command=result.x[0];out.append(command);eff=decay*eff+(1-decay)*command;prev=command;guess=np.r_[result.x[1:],result.x[-1]]
  if rd==8:check.append([command-row[8],err[0]-row[6],wrap(err[1]-row[7]),nearest-row[5]])
 arr=np.array(out);outputs[str(rd)]=arr.tolist()
 print('rd',rd,'solver failures',fails)
 for name,start,end in [('straight',1790188135,1790188155),('curve',1790188167,1790188185),('all',tr[0,0],tr[-1,0])]:
  mask=(tr[:,0]>=start)&(tr[:,0]<end)&(tr[:,10]==1);u=arr[mask]
  if len(u):print(name,'count',len(u),'rms_deg',np.sqrt(np.mean(np.degrees(u)**2)),'tv_deg',sum(abs(np.degrees(np.diff(u)))),'sat',np.mean(abs(u)>.299))
 if check:print('BASELINE abs max error steer/ey/epsi/index',np.max(abs(np.array(check)),axis=0))
(root/'damping_comparison.json').write_text(json.dumps(dict(t=tr[:,0].tolist(),recorded=tr[:,8].tolist(),outputs=outputs)))
