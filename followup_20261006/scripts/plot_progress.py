import pickle, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
SER=pickle.load(open('../runs_series.pkl','rb'))
fails={'A_1003_0559_tau005':[1790974782.712,1790974786.715,1790974787.714,1790974791.714,1790974795.712],
       'R2_1005_2344_sc1':[1791211485.894],'R3_1005_2345_sc1':[1791211540.405]}
cols={'A_1003_0559_tau005':'tab:red','R2_1005_2344_sc1':'tab:orange','R3_1005_2345_sc1':'tab:green'}
fig,ax=plt.subplots(4,1,figsize=(15,13),sharex=True)
for k,c in cols.items():
    s=SER[k]; pc=np.interp(s['tc'],s['lp_t'],s['prog']); ph=np.interp(s['th'],s['lp_t'],s['prog'])
    mc=(s['tc']>=s['t_on'])&(s['tc']<=s['t_off']); mh=(s['th']>=s['t_on'])&(s['th']<=s['t_off'])
    ax[0].plot(pc[mc],np.degrees(s['sc'][mc]),color=c,lw=.8,label=k+' cmd')
    ax[1].plot(ph[mh],np.degrees(s['deff'][mh]),color=c,lw=1.2,label=k+' SIH effective steer')
    ax[2].plot(ph[mh],np.degrees(s['yr'][mh]),color=c,label=k)
    ax[3].plot(ph[mh],s['v'][mh],color=c,label=k)
    for tf in fails[k]:
        pf=np.interp(tf,s['lp_t'],s['prog'])
        for a in ax: a.axvline(pf,color=c,ls='--',alpha=.6)
for a,l in zip(ax,['steer cmd deg (MAVLink sign)','SIH effective steer deg','SIH yaw rate deg/s','SIH speed m/s']): a.set_ylabel(l); a.grid(True); a.axvline(88,color='k',ls=':')
ax[0].axhline(17.19,color='k',lw=.5); ax[0].axhline(-17.19,color='k',lw=.5)
ax[0].legend(fontsize=8); ax[3].set_xlabel('progress along scenario1 global path (m); dashed = upper QP failure; dotted = junction ~88 m')
plt.tight_layout(); plt.savefig('../figures/progress_A_R2_R3.png',dpi=90)
