from pathlib import Path
import json,pickle,numpy as np
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;ws=p.parents[2];runs=['20261003_055920'];results={};curves={}
for run in runs:
 root=ws/('drive_debug_'+run+'_light')/'analysis';d=pickle.load(open(root/'decoded.pkl','rb'));r=json.loads((root/'telemetry.json').read_text());lo=np.array([a['data'] for a in d['/debug/lower_mpc_trace']]);h=[a for a in r if a.get('mavpackettype',a.get('type'))=='HIL_STATE_QUATERNION'];t=np.array([a['t'] for a in h]);v=np.array([[a['vx'],a['vy']] for a in h])*.01;q=np.array([a['attitude_quaternion'] for a in h]);yaw=np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2));beta=np.arctan2(np.sin(np.arctan2(v[:,1],v[:,0])-yaw),np.cos(np.arctan2(v[:,1],v[:,0])-yaw));delta=-np.arctan(2*np.tan(beta));moving=lo[(lo[:,10]==1)&(lo[:,4]>3)];begin=moving[0,0];finish=moving[-1,0];mask=(t>=begin)&(t<=finish)&(np.linalg.norm(v,axis=1)>3);t=t[mask];delta=delta[mask];t0=t[0];tt=t-t0;ct=lo[:,0]-t0;cmd=lo[:,26];train=(tt>1)&(tt<tt[-1]*.55);test=tt>=tt[-1]*.55
 def response(params):
  gain,tau,delay=params;vals=cmd[np.clip(np.searchsorted(ct,tt-delay,side='right')-1,0,len(ct)-1)];pred=np.zeros(len(tt));pred[0]=delta[0]
  for i in range(1,len(tt)):
   decay=np.exp(-(tt[i]-tt[i-1])/tau);pred[i]=decay*pred[i-1]+(1-decay)*gain*vals[i-1]
  return pred
 fit=differential_evolution(lambda z:np.mean((response(z)[train]-delta[train])**2),[(.3,1.5),(.005,1.5),(0,.5)],seed=4,popsize=8,maxiter=85,tol=1e-7);best=response(fit.x);cfg=response([1,.05,0]);rmse=lambda a,m:float(np.degrees(np.sqrt(np.mean((a[m]-delta[m])**2))))
 results[run]={'gain_tau_delay':fit.x.tolist(),'train_duration_s':float(tt[-1]*.55),'fitted_rmse_train_deg':rmse(best,train),'fitted_rmse_test_deg':rmse(best,test),'configured_rmse_train_deg':rmse(cfg,train),'configured_rmse_test_deg':rmse(cfg,test),'command_std_train_deg':float(np.degrees(np.std(np.interp(tt[train],ct,cmd)))),'logged_model_delay_tau':np.unique(lo[:,22:24],axis=0).tolist()};curves[run]=(tt,delta,ct,cmd,best)
 print(run,results[run],flush=True)
(p/'response_comparison.json').write_text(json.dumps(results,indent=2))
