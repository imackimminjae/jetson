from pathlib import Path
import json,re
import numpy as np
from scipy.optimize import least_squares
from datetime import datetime
root=Path(__file__).resolve().parent;d=np.load(root/'telemetry.npz');h=d['truth'];a=d['att'];cmd=d['cmd'];p=d['position'];w,x,y,z=h[:,7:11].T
yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));course=np.arctan2(h[:,5],h[:,4]);beta=np.arctan2(np.sin(course-yaw),np.cos(course-yaw));v=np.hypot(h[:,4],h[:,5]);forward=np.cos(yaw)*h[:,4]+np.sin(yaw)*h[:,5];lateral=-np.sin(yaw)*h[:,4]+np.cos(yaw)*h[:,5];goal=1790186362.954
active=(h[:,0]>=1790186294)&(h[:,0]<goal)&(v>1)
summary={}
for name,data in [('beta_deg',np.degrees(beta)),('lateral_mps',lateral),('speed_mps',v),('yaw_rate_deg_s',np.degrees(h[:,3]))]:
 q=np.quantile(abs(data[active]),[.5,.95,1]);summary[name]=q.tolist();print(name,'abs median/p95/max',q)
att_yaw=np.interp(h[:,0],a[:,0],np.unwrap(a[:,3]));err=np.arctan2(np.sin(att_yaw-yaw),np.cos(att_yaw-yaw));summary['EKF_yaw_error_deg']=np.degrees(np.quantile(abs(err[active]),[.5,.95,1])).tolist();print('EKF yaw error',summary['EKF_yaw_error_deg'])
for tt in np.arange(1790186300,goal,10):
 m=(h[:,0]>=tt)&(h[:,0]<tt+10)&active
 if m.any():print(datetime.fromtimestamp(tt),'maxbeta',max(abs(np.degrees(beta[m]))),'maxrate',max(abs(np.degrees(h[m,3]))),'speedmed',np.median(v[m]))
# Uniform resampling for comparing equivalent steering responses. Skip low speed.
dt=.01;t=np.arange(1790186292,goal,dt);speed=np.interp(t,h[:,0],v);rate=np.interp(t,h[:,0],h[:,3]);mask=(speed>1)&(t>1790186298)
steer_observed=np.arctan(2.8*rate/np.maximum(speed,.1))
# First-order model driven by zero-order-held logged command; gain is radians/unit.
def response(delay,tau,gain):
 index=np.clip(np.searchsorted(cmd[:,0],t-delay,side='right')-1,0,len(cmd)-1);u=cmd[index,2]*gain;out=np.zeros(len(t));decay=np.exp(-dt/max(tau,1e-6))
 for i in range(1,len(t)):out[i]=decay*out[i-1]+(1-decay)*u[i-1]
 return out
current=response(.3,1.,.3);current_rate=speed/2.8*current
summary['current_model_yaw_rate_rmse_deg_s']=float(np.sqrt(np.mean(np.degrees(current_rate[mask]-rate[mask])**2)))
print('current delay .3 tau1 gain.3 yawrateRMSEdeg/s',summary['current_model_yaw_rate_rmse_deg_s'])
# Delay search cannot rely on a finite-difference optimizer through ZOH indices.
best=None
for delay in np.arange(0,.61,.01):
 def res(z):return (response(delay,z[0],z[1])-steer_observed)[mask]
 fit=least_squares(res,[.2,.35],bounds=([.001,.05],[2.,.7]),max_nfev=25)
 mse=np.mean(fit.fun**2)
 if best is None or mse<best[0]:best=(mse,float(delay),float(fit.x[0]),float(fit.x[1]))
summary['equivalent_response_fit']={'delay':best[1],'tau':best[2],'gain':best[3],'steering_rmse_rad':np.sqrt(best[0])};print('fit',summary['equivalent_response_fit'])
np.savez(root/'comparison.npz',t=h[:,0],yaw=yaw,course=course,beta=beta,speed=v,lateral=lateral,active=active,model_t=t,model_rate=current_rate,truth_rate=rate,model_delta=current,observed_delta=steer_observed,fitted_delta=response(*best[1:]))
for offset in [0,7,15]:
 i=np.argmin(abs(p[:,0]-goal-offset));j=np.argmin(abs(p[:,0]-goal));print('goal+',offset,'speed',np.linalg.norm(p[i,4:6]),'displacement',np.linalg.norm(p[i,1:3]-p[j,1:3]))
(root/'summary.json').write_text(json.dumps(summary,indent=2))
