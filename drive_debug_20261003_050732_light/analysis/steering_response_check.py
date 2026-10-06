import json,pickle,numpy as np
from pathlib import Path
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;r=json.loads((p/'telemetry.json').read_text());d=pickle.load(open(p/'decoded.pkl','rb'));lo=np.array([a['data'] for a in d['/debug/lower_mpc_trace']]);h=[a for a in r if a['mavpackettype']=='HIL_STATE_QUATERNION'];t=np.array([a['t'] for a in h]);v=np.array([[a['vx'],a['vy']] for a in h])*.01;q=np.array([a['attitude_quaternion'] for a in h]);yaw=np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2));beta=np.arctan2(np.sin(np.arctan2(v[:,1],v[:,0])-yaw),np.cos(np.arctan2(v[:,1],v[:,0])-yaw));delta=-np.arctan(2*np.tan(beta));mask=(t>=lo[0,0])&(t<=lo[-1,0])&(np.linalg.norm(v,axis=1)>3);t=t[mask];delta=delta[mask];t0=t[0];tt=t-t0;ct=lo[:,0]-t0;cmd=lo[:,26]
# Fit an equivalent first-order response for diagnosis; not a firmware parameter estimate.
def response(params):
 gain,tau,delay=params;u=cmd[np.clip(np.searchsorted(ct,tt-delay,side='right')-1,0,len(ct)-1)];pred=np.zeros(len(tt));pred[0]=delta[0]
 for i in range(1,len(tt)):
  decay=np.exp(-(tt[i]-tt[i-1])/tau);pred[i]=decay*pred[i-1]+(1-decay)*gain*u[i-1]
 return pred
mid=12.;train=(tt>1)&(tt<mid);test=(tt>=mid)
fit=differential_evolution(lambda z:np.mean((response(z)[train]-delta[train])**2),[(.3,1.5),(.01,1.5),(0,.7)],seed=4,popsize=8,maxiter=70,tol=1e-7);best=response(fit.x);configured=response([1,.05,0]);rmse=lambda a,m:float(np.degrees(np.sqrt(np.mean((a[m]-delta[m])**2))))
metrics={'assumption':'no-slip bicycle; published velocity at vehicle center, rear axle offset 1.4m, wheelbase 2.8m; inferred steering is atan(2*tan(course-heading)), sign changed to ROS','fit_training_window_s':[1,mid],'equivalent_gain_tau_s_delay_s':fit.x.tolist(),'configured_model_rmse_deg_train':rmse(configured,train),'configured_model_rmse_deg_test':rmse(configured,test),'fitted_model_rmse_deg_train':rmse(best,train),'fitted_model_rmse_deg_test':rmse(best,test),'warning':'Effective response inferred from motion, not direct steering feedback; gain/delay/tau are not uniquely identified firmware values.'}
(p/'steering_response_metrics.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
fig,ax=plt.subplots(figsize=(13,4.8));ax.step(ct,np.degrees(cmd),where='post',lw=1.3,color='#c62828',label='조향 출력 명령');ax.plot(tt,np.degrees(delta),color='#087e8b',lw=1.6,label='SIH 이동방향·차체방향으로 역산한 유효 조향');ax.plot(tt,np.degrees(best),color='#7b1fa2',ls='--',alpha=.8,label=f'등가 응답 적합: 이득 {fit.x[0]:.2f}, 시정수 {fit.x[1]:.2f}s, 지연 {fit.x[2]:.2f}s');ax.axvline(mid,c='gray',ls=':',label='왼쪽 구간으로 적합 / 오른쪽 구간으로 확인');ax.set(xlim=(0,tt[-1]),xlabel='기록 시작부터 시간 (초)',ylabel='조향 (도)',title='조향 명령 대비 SIH의 유효 반응 — 직접 조향 센서 측정은 아님\n차체 중심·뒷차축 거리 1.4m의 무슬립 자전거 모델 가정');ax.grid(alpha=.2);ax.legend(fontsize=9,loc='upper left');fig.tight_layout();fig.savefig(p/'steering_response.png',dpi=160)
