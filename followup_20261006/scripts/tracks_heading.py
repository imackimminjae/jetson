import pickle, numpy as np, time, json
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from analyze_runs import ROUTES, clock, wrap, q2yaw, load, arr, RUNS, ts
SER=pickle.load(open('../runs_series.pkl','rb'))
fig,ax=plt.subplots(1,2,figsize=(18,10))
cols={'A_1003_0559_tau005':'tab:red','B_1003_0639_tau074':'tab:blue','R2_1005_2344_sc1':'tab:orange','R3_1005_2345_sc1':'tab:green','R1_1005_2341_sc3':'tab:purple'}
GP=ROUTES['scenario1']; ax[0].plot(GP[:,0],GP[:,1],'k--o',label='scenario1 global path')
GP3=ROUTES['scenario3']; ax[1].plot(GP3[:,0],GP3[:,1],'k--o',label='scenario3 global path')
out={}
for k,s in SER.items():
    m=(s['lp_t']>=s['t_on']-1)&(s['lp_t']<=s['t_off']+1)
    a=ax[1] if s['route']=='scenario3' else ax[0]
    a.plot(s['mx'][m],s['my'][m],color=cols[k],label=k)
    for dt in range(0,45,5):
        t=s['t_on']+dt
        if t<=s['t_off']: a.annotate(f'{dt}',(np.interp(t,s['lp_t'],s['mx']),np.interp(t,s['lp_t'],s['my'])),fontsize=7,color=cols[k])
    # EKF yaw (ATTITUDE) vs SIH truth yaw (HIL_STATE)
    cfg=RUNS[k]; D=load(cfg['pkl']); a0,b0=s['t_on']-5,s['t_off']
    att=arr(D,'ATTITUDE',a0,b0,lambda d:(d['_t'],d['yaw'])); hs=arr(D,'HIL_STATE_QUATERNION',a0,b0,lambda d:(d['_t'],q2yaw(d['attitude_quaternion'])))
    dy=np.degrees(wrap(np.interp(att[:,0],hs[:,0],np.unwrap(hs[:,1]))-att[:,0]*0-att[:,1]))
    out[k]=dict(ekf_minus_sih_yaw_deg_p50=round(float(-np.median(dy)),2),p05_p95=[round(float(-np.quantile(dy,.95)),2),round(float(-np.quantile(dy,.05)),2)])
for a in ax: a.set_aspect('equal'); a.grid(True); a.legend(fontsize=8)
ax[0].set_title('scenario1 runs (numbers = s since throttle on)'); ax[1].set_title('scenario3 run R1')
plt.tight_layout(); plt.savefig('../figures/tracks_all.png',dpi=90)
print(json.dumps(out,indent=1))
