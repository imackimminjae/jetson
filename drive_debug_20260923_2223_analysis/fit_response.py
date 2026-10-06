import numpy as np,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
from scipy.optimize import least_squares
r=np.load(ROOT/'response_arrays.npz');dt=.02;t=np.arange(1790169810,1790169863,dt);v=np.interp(t,r['t'],r['v']);omega=np.interp(t,r['t'],r['yr']);cmd=r['cmd'];u=cmd[:,2]
fit=(t>=1790169812)&(t<1790169830);test=(t>=1790169830)&(t<1790169862)
def simulate(p):
 delay,tau,gain=p;indices=np.clip(np.searchsorted(cmd[:,0],t-delay,side='right')-1,0,len(cmd)-1);delta=gain*u[indices];y=np.zeros(len(t));y[0]=omega[0]
 for i in range(1,len(t)):y[i]=y[i-1]+dt/tau*(v[i-1]/2.8*np.tan(delta[i-1])-y[i-1])
 return y
best=None
for delay in [0,.05,.1,.2,.3,.4]:
 result=least_squares(lambda p:(simulate([delay,p[0],p[1]])-omega)[fit],[.8,.3],bounds=([.05,.15],[3,.6]));p=[delay,*result.x];pred=simulate(p);loss=np.mean((pred[fit]-omega[fit])**2)
 if best is None or loss<best[0]:best=(loss,p,pred)
 print('delay tau gain',p,'fit',np.sqrt(loss),'heldout',np.sqrt(np.mean((pred[test]-omega[test])**2)))
loss,p,pred=best
indices=np.clip(np.searchsorted(cmd[:,0],t,side='right')-1,0,len(cmd)-1);instant=v/2.8*np.tan(.30*u[indices]);print('instant errors',*[np.sqrt(np.mean((instant[m]-omega[m])**2)) for m in [fit,test]])
json.dump({'model':'yaw_rate_first_order','delay_sec':p[0],'tau_sec':p[1],'steer_gain_rad':p[2],'fit_rms_radps':float(np.sqrt(loss)),'heldout_rms_radps':float(np.sqrt(np.mean((pred[test]-omega[test])**2)))},open(ROOT/'response_fit.json','w'),indent=2)
