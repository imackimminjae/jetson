from pathlib import Path
import json,pickle,numpy as np
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;ws=p.parents[2];runs=['20261002_041101','20261002_041646','20261002_043054','20261003_050732','20261003_051409'];results={};curves={}
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
fig,axs=plt.subplots(2,1,figsize=(13,8));
for ax,run,title in zip(axs,['20261002_041646','20261003_051409'],['어제 안정적이었던 1Hz 주행 (04:16)','최신 1Hz 주행 (05:14)']):
 tt,delta,ct,cmd,best=curves[run];g,tau,delay=results[run]['gain_tau_delay'];ax.step(ct,np.degrees(cmd),where='post',c='#c62828',lw=1.2,label='조향 명령');ax.plot(tt,np.degrees(delta),c='#087e8b',label='SIH 유효 조향 (움직임으로 역산)');ax.plot(tt,np.degrees(best),'--',c='#7b1fa2',alpha=.7,label=f'등가 응답: 시정수 {tau:.2f}s, 지연 {delay:.2f}s');ax.set(xlim=(0,tt[-1]),ylim=(-19,19),ylabel='조향 (도)',xlabel='해당 기록의 이동 구간부터 시간 (초)',title=title);ax.legend(fontsize=9);ax.grid(alpha=.2)
fig.suptitle('같은 1Hz라도 실제 조향 반응이 달라졌는지 비교\n무슬립 자전거·차체 중심 기준 가정 / 앞 55% 적합, 뒤 45% 검증',fontsize=14);fig.tight_layout();fig.savefig(p/'response_comparison.png',dpi=160)
