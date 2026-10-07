import numpy as np, pickle, json, time
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
L=2.8; LR=1.4
res = pickle.load(open('traj_res.pkl','rb'))
def lpf(t,u,tau):
    y=np.zeros_like(u)
    for i in range(1,len(t)):
        a=1-np.exp(-(t[i]-t[i-1])/tau); y[i]=y[i-1]+a*(u[i]-y[i-1])
    return y
out={}
fig,axs=plt.subplots(4,2,figsize=(18,14),sharex='col')
for ci,(name,key) in enumerate((('A_0559_tau0.05','A'),('B_0639_tau0.74','B'))):
    S=np.load(f'series_{name}.npz'); R=res[key]
    tc,sc,thr,th,v,deff,yr = S['tc'],S['steer_cmd'],S['thr'],S['th'],S['v'],S['deff'],S['yr']
    prog_t = np.interp(th, R['t'], R['prog']); lat_t = np.interp(th, R['t'], R['lat'])
    t0 = th[v>1.0][0]
    # common segment: progress 0..88 m (before the branch near (108,-59)); and the following 88..end
    for segname,(p0,p1) in (('common_0_88m',(0,88)),('after_88m',(88,1e9))):
        m = (prog_t>=p0)&(prog_t<p1)&(v>1.0)
        if m.sum()<10: continue
        t_a, t_b = th[m][0], th[m][-1]
        mc = (tc>=t_a)&(tc<=t_b)
        u = np.interp(th, tc, sc, left=0.0)
        r = dict(clock=[clock(t_a),clock(t_b)], dur=round(t_b-t_a,2), speed_p50=round(float(np.median(v[m])),2),
                 cmd_abs_p95_deg=round(float(np.degrees(np.quantile(np.abs(sc[mc]),.95))),2),
                 cmd_sat_frac=round(float(np.mean(np.abs(sc[mc])>=0.298)),4),
                 truth_eff_steer_abs_p95_deg=round(float(np.degrees(np.quantile(np.abs(deff[m]),.95))),2),
                 yawrate_abs_p95_degps=round(float(np.degrees(np.quantile(np.abs(yr[m]),.95))),2),
                 lat_dev_global_abs_p50_p95_m=[round(float(np.quantile(np.abs(lat_t[m]),q)),2) for q in (.5,.95)],
                 opp_sign_frac=round(float(np.mean((np.sign(u[m])*np.sign(deff[m])<0)&(np.abs(u[m])>np.radians(3)))),4),
                 model005_rmse_deg=round(float(np.degrees(np.sqrt(np.mean((lpf(th,u,0.05)[m]-deff[m])**2)))),3),
                 model074_rmse_deg=round(float(np.degrees(np.sqrt(np.mean((lpf(th,u,0.74)[m]-deff[m])**2)))),3))
        # distinct command updates + reversals
        scu = sc[mc]; tcu = tc[mc]; keep = np.r_[True, np.abs(np.diff(scu))>1e-6]; scu=scu[keep]
        r['cmd_reversals_per_s'] = round(float(np.sum(np.sign(scu[1:])*np.sign(scu[:-1])<0)/(t_b-t_a)),3)
        r['cmd_step_p95_deg'] = round(float(np.degrees(np.quantile(np.abs(np.diff(scu)),.95))),2)
        # oscillation energy: std of high-pass (cmd - 1s moving avg)
        k=max(1,int(1.0/np.median(np.diff(tcu)))); ma=np.convolve(sc[mc],np.ones(k)/k,'same'); r['cmd_hf_std_deg']=round(float(np.degrees(np.std(sc[mc]-ma))),2)
        # best tau on segment
        best=None
        for tau in np.arange(0.1,1.6,0.025):
            yp=lpf(th,u,tau); g=float(np.dot(yp[m],deff[m])/max(1e-9,np.dot(yp[m],yp[m]))); e=np.sqrt(np.mean((g*yp[m]-deff[m])**2))
            if best is None or e<best[0]: best=(e,tau,g)
        r['seg_fit_tau_gain_rmse']=[round(best[1],3),round(best[2],3),round(float(np.degrees(best[0])),3)]
        out[f'{key}:{segname}']=r
    # plots
    ax=axs[0,ci]; ax.plot(tc-t0,np.degrees(sc),'k',lw=.8,label='steer cmd (MAVLink param2*0.35)'); ax.plot(th-t0,np.degrees(deff),'tab:green',label='SIH truth eff. steer (from yaw rate)')
    u=np.interp(th,tc,sc,left=0); ax.plot(th-t0,np.degrees(lpf(th,u,0.05)),'tab:orange',lw=.8,alpha=.8,label='1st-order tau=0.05 pred'); ax.plot(th-t0,np.degrees(lpf(th,u,0.74)),'tab:blue',lw=.8,alpha=.8,label='1st-order tau=0.74 pred')
    ax.axhline(17.19,color='r',ls=':'); ax.axhline(-17.19,color='r',ls=':'); ax.set_title(f'run {key} ({name})'); ax.legend(fontsize=7); ax.grid(True); ax.set_ylabel('deg')
    ax=axs[1,ci]; ax.plot(th-t0,np.degrees(yr),'tab:purple'); ax.set_ylabel('truth yaw rate deg/s'); ax.grid(True)
    ax=axs[2,ci]; ax.plot(th-t0,v,'tab:green',label='truth speed'); ax.plot(R['t']-t0,R['v_ekf'],'tab:gray',lw=.6,label='EKF speed'); ax.plot(tc-t0,thr*10,'k',lw=.6,label='throttle x10'); ax.legend(fontsize=7); ax.grid(True)
    ax=axs[3,ci]; ax.plot(th-t0,prog_t,'tab:brown',label='progress along global path'); ax.axhline(88,color='k',ls=':'); ax.legend(fontsize=7); ax.grid(True); ax.set_xlabel('s since moving')
    if key=='A':
        for tf in (1790974782.712,1790974786.715,1790974787.714,1790974791.714,1790974795.712):
            for row in range(4): axs[row,ci].axvline(tf-t0,color='r',alpha=.3)
    else:
        for row in range(4): axs[row,ci].axvline(1790977198.815-t0,color='r',alpha=.3)
plt.tight_layout(); plt.savefig('timeseries_AB.png',dpi=100)
json.dump(out,open('segment_compare.json','w'),indent=1)
for k,v in out.items(): print(k, json.dumps(v))
