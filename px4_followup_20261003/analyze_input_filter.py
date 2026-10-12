"""Offline reconstruction from actual actuator_outputs_sim and groundtruth ULog."""
import argparse,json
from pathlib import Path
import numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar

p=argparse.ArgumentParser();p.add_argument('ulog',type=Path);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
args.output.mkdir(exist_ok=True,parents=True)
u=ULog(str(args.ulog));d={v.name:v.data for v in u.data_list if v.multi_id==0}
g=d['vehicle_local_position_groundtruth'];a=d['actuator_outputs_sim'];origin=float(g['timestamp'][0])*1e-6
t=g['timestamp']*1e-6-origin;at=a['timestamp']*1e-6-origin
beta=np.arctan2(np.sin(np.arctan2(g['vy'],g['vx'])-g['heading']),np.cos(np.arctan2(g['vy'],g['vx'])-g['heading']))
delta=np.arctan(2*np.tan(beta));speed=np.hypot(g['vx'],g['vy'])
hz=float(u.initial_parameters['IMU_GYRO_RATEMAX']);tau=float(u.initial_parameters['SIH_T_TAU']);dt=1/hz
st=np.arange(t[0],t[-1]+dt*.5,dt);idx=np.clip(np.searchsorted(at,st,side='right')-1,0,len(at)-1)
sp=a['output[1]'][idx];age=st-at[idx];scale=.35
def predict(mode,alpha=None):
 x=np.zeros(len(st));x[0]=delta[0]/scale
 for i in range(1,len(st)):
  if age[i]>.5 or age[i]<0:x[i]=0.;continue
  x[i]=x[i-1]
  if mode=='continuous' or idx[i]!=idx[i-1]:
   k=-np.expm1(-dt/tau) if mode=='continuous' else (dt/tau if alpha is None else alpha)
   x[i]+=k*(sp[i]-x[i])
 return np.interp(t,st,scale*x)
event_pred=predict('event');cont_pred=predict('continuous')
valid=(t>1)&(speed>1)&(g['v_xy_valid']==1)
rmse=lambda v:float(np.degrees(np.sqrt(np.mean((v[valid]-delta[valid])**2))))
fit=minimize_scalar(lambda alpha:rmse(predict('event',alpha)),bounds=(.005,.15),method='bounded')
gaps=[]
for i in np.flatnonzero(np.diff(at)>.1):
 start,end=float(at[i]),float(at[i+1]);threshold=start+.5
 sample=lambda when:float(np.degrees(delta[min(np.searchsorted(t,when),len(t)-1)]))
 gaps.append({'start_s':start,'end_s':end,'duration_s':end-start,'timeout_threshold_s':threshold,
              'steering_input_before':float(a['output[1]'][i]),'steering_deg_before_timeout':sample(threshold-.025),
              'steering_deg_after_timeout':sample(threshold+.025),
              'speed_m_s_at_timeout':float(np.interp(threshold,t,speed))})
normal=np.diff(at);normal=normal[(normal>.00001)&(normal<.1)]
result={'duration_s':float(t[-1]),'sih_hz_parameter':hz,'tau_parameter_s':tau,'fixed_alpha':dt/tau,
        'steering_scale_assumption_rad':scale,'physical_assumption':'No-slip bicycle; center reference; wheelbase2.8m; rear offset1.4m.',
        'continuous_005_rmse_deg':rmse(cont_pred),'event_gated_fixed_rmse_deg':rmse(event_pred),
        'event_gated_fitted_alpha':float(fit.x),'fitted_rmse_deg':float(fit.fun),
        'normal_output_rate_hz':float(1/np.mean(normal)),'output_gap_percentiles_s':np.percentile(np.diff(at),[0,50,95,100]).tolist(),
        'gaps':gaps,'dropouts':len(u.dropouts),'corruption':bool(u.file_corruption),
        'limitations':'Fixed replay assumes 0.35rad steering scale and uniform SIH period; uORB processing occurs after publication; not an exact CPU instruction trace.'}
(args.output/'input_filter_results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
axs[0].step(at,np.degrees(scale*a['output[1]']),where='post',color='#c62828',alpha=.5,label='Raw steering input (0.35rad scale)')
axs[0].plot(t,np.degrees(delta),color='#087e8b',label='Groundtruth equivalent steering')
axs[0].plot(t,np.degrees(event_pred),'--',color='#7b1fa2',label='Original event-gated filter + 0.5s watchdog')
axs[0].set_ylabel('Steering (deg)');axs[0].legend(fontsize=8)
axs[1].plot(st,age,color='#555555');axs[1].axhline(.5,color='#c62828',ls='--');axs[1].set_ylabel('Actuator age (s)')
axs[2].plot(t,speed,label='SIH speed (m/s)')
for name,field,label in [('vehicle_land_detected','at_rest','Land at_rest'),('estimator_status_flags','cs_vehicle_at_rest','EKF at_rest')]:
 s=d[name];axs[2].step(s['timestamp']*1e-6-origin,s[field],where='post',label=label)
axs[2].legend(fontsize=8);axs[2].set_xlabel('Time from first groundtruth sample (s)')
for ax in axs:
 ax.set_xlim(0,t[-1]);ax.grid(alpha=.2)
 for gap in gaps:ax.axvspan(gap['timeout_threshold_s'],gap['end_s'],alpha=.12,color='#c62828')
fig.suptitle('Recovered ULog: actual input updates, steering response and rest flags')
fig.tight_layout();fig.savefig(args.output/'input_filter_evidence.png',dpi=155)
