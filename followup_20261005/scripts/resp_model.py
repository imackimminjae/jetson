import pickle, numpy as np, time, json
D = pickle.load(open('tlog_windows.pkl','rb'))
def ts(s): return time.mktime(time.strptime(s, '%Y-%m-%d %H:%M:%S'))
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
L=2.8; LR=1.4; SCALE=0.35
runs = {'A_0559_tau0.05': (ts('2026-10-03 05:59:15'), ts('2026-10-03 06:00:05')),
        'B_0639_tau0.74': (ts('2026-10-03 06:39:12'), ts('2026-10-03 06:40:12'))}
def arr(typ, a, b, f):
    return np.array([f(d) for d in D[typ] if a <= d['_t'] <= b])
def quat_to_yaw(q):  # q = [w,x,y,z] NED/FRD
    w,x,y,z = q
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def eff_steer(yawrate, v):
    # center-reference kinematic bicycle: psi_dot = v_c*cos(beta)*tan(delta)/L, beta=atan(LR*tan(delta)/L)
    d = np.arctan2(L*yawrate, np.maximum(v,0.3))
    for _ in range(10):
        beta = np.arctan(LR*np.tan(d)/L)
        d = np.arctan(L*yawrate/(np.maximum(v,0.3)*np.cos(beta)))
    return d
def lpf(t, u, tau, delay=0.0, gain=1.0):
    # continuous first order on an irregular grid, with input delay
    y = np.zeros_like(u); ud = np.interp(t-delay, t, u, left=u[0])
    for i in range(1,len(t)):
        a = 1-np.exp(-(t[i]-t[i-1])/tau); y[i] = y[i-1] + a*(gain*ud[i]-y[i-1])
    return y
out = {}
for name,(a,b) in runs.items():
    cl = arr('COMMAND_LONG', a, b, lambda d: (d['_t'], d['command'], d['param1'], d['param2'], d['target_system']))
    cl = cl[cl[:,1]==187]
    ack = arr('COMMAND_ACK', a, b, lambda d: (d['_t'], d['command'], d['result']))
    hac = arr('HIL_ACTUATOR_CONTROLS', a, b, lambda d: (d['_t'], d['time_usec']*1e-6, *d['controls'][:4], d['mode'], d['flags']))
    hs = arr('HIL_STATE_QUATERNION', a, b, lambda d: (d['_t'], d['time_usec']*1e-6, quat_to_yaw(d['attitude_quaternion']), d['yawspeed'], d['vx']/100, d['vy']/100, d['lat']*1e-7, d['lon']*1e-7, d['xacc'], d['yacc']))
    lp = arr('LOCAL_POSITION_NED', a, b, lambda d: (d['_t'], d['time_boot_ms']*1e-3, d['x'], d['y'], d['vx'], d['vy']))
    att = arr('ATTITUDE', a, b, lambda d: (d['_t'], d['yaw'], d['yawspeed']))
    mc = arr('MANUAL_CONTROL', a, b, lambda d: (d['_t'], d['x'], d['y'], d['z'], d['r'], d['target']))
    # active drive window: throttle cmd>0 first..last
    act = cl[cl[:,2]>0]
    t_on, t_off = act[0,0], act[-1,0]
    r = {'window': [clock(a), clock(b)], 'cmd187_count': len(cl), 'throttle_active': [clock(t_on), clock(t_off)], 'active_sec': round(t_off-t_on,2)}
    dtc = np.diff(cl[:,0]); r['cmd187_rate_hz'] = round(1/np.median(dtc),2); r['cmd187_max_gap_s'] = round(dtc.max(),3)
    r['cmd187_gaps_over_0.3s'] = [(clock(cl[i,0]), round(g,3)) for i,g in enumerate(dtc) if g>0.3]
    r['ack_count'] = len(ack); r['ack_results'] = {int(k):int(v) for k,v in zip(*np.unique(ack[:,2], return_counts=True))}
    dth = np.diff(hac[:,0]); r['hil_act_rate_hz'] = round(1/np.median(dth),2); r['hil_act_max_gap_s'] = round(dth.max(),3)
    r['hil_act_gaps_over_0.3s'] = [(clock(hac[i,0]), round(g,3)) for i,g in enumerate(dth) if g>0.3]
    r['manual_control_count'] = len(mc); r['manual_control_rate_hz'] = round(1/np.median(np.diff(mc[:,0])),2) if len(mc)>2 else None
    r['manual_control_sample'] = mc[len(mc)//2].tolist() if len(mc) else None
    # steering command in rad
    sel = (cl[:,0]>=t_on)&(cl[:,0]<=t_off)
    tc = cl[sel,0]; steer_cmd = cl[sel,3]*SCALE; thr = cl[sel,2]
    r['steer_cmd_abs_p50_p95_max_deg'] = [round(np.degrees(x),2) for x in np.quantile(np.abs(steer_cmd),[.5,.95,1])]
    r['steer_cmd_saturation_frac(|d|>=0.298)'] = round(float(np.mean(np.abs(steer_cmd)>=0.298)),4)
    # sign reversals among distinct consecutive commands (dedupe repeats from keepalive)
    uniq_idx = np.r_[True, np.abs(np.diff(steer_cmd))>1e-6]
    sc_u = steer_cmd[uniq_idx]; tc_u = tc[uniq_idx]
    rev = np.sum(np.sign(sc_u[1:])*np.sign(sc_u[:-1])<0)
    r['steer_cmd_distinct_updates'] = int(len(sc_u)); r['distinct_update_rate_hz']=round(len(sc_u)/(t_off-t_on),2)
    r['steer_cmd_sign_reversals'] = int(rev); r['reversals_per_sec'] = round(rev/(t_off-t_on),3)
    r['steer_cmd_step_abs_p50_p95_deg'] = [round(np.degrees(x),2) for x in np.quantile(np.abs(np.diff(sc_u)),[.5,.95])]
    r['throttle_p50_p95'] = [round(float(x),3) for x in np.quantile(thr,[.5,.95])]
    # truth
    hsel = (hs[:,0]>=t_on)&(hs[:,0]<=t_off+1.0)
    th = hs[hsel,0]; yaw = np.unwrap(hs[hsel,2]); yr = hs[hsel,3]; v = np.hypot(hs[hsel,4],hs[hsel,5])
    r['truth_speed_p50_p95_max'] = [round(float(x),3) for x in np.quantile(v,[.5,.95,1])]
    r['truth_yawrate_abs_p95_degps'] = round(float(np.degrees(np.quantile(np.abs(yr),.95))),2)
    deff = eff_steer(yr, v)
    moving = v>1.0
    r['truth_eff_steer_abs_p95_max_deg'] = [round(np.degrees(x),2) for x in np.quantile(np.abs(deff[moving]),[.95,1])]
    # yaw-rate reversal count (oscillation) using smoothed eff steer
    k = max(1,int(0.2/np.median(np.diff(th))))
    ds = np.convolve(deff, np.ones(k)/k, mode='same')
    zr = np.sum((np.sign(ds[1:])*np.sign(ds[:-1])<0)&moving[1:])
    r['truth_eff_steer_zero_crossings'] = int(zr)
    # hil actuator steering (controls[1]) hold vs cmd: compare cmd at 30Hz with HIL act at ~26Hz
    hsel2 = (hac[:,0]>=t_on)&(hac[:,0]<=t_off)
    ta = hac[hsel2,0]; sa = hac[hsel2,3]*SCALE; thr_a = hac[hsel2,2]
    cmd_on_act = np.interp(ta, tc, steer_cmd)
    r['hil_act_steer_minus_cmd_rmse_deg'] = round(float(np.degrees(np.sqrt(np.mean((sa-cmd_on_act)**2)))),3)
    r['hil_act_controls_sample'] = hac[len(hac)//2,2:6].round(3).tolist()
    # ---- first-order identification: truth eff steer vs cmd ----
    tt = th[moving]; y = deff[moving]
    u_full = np.interp(th, tc, steer_cmd, left=0.0)
    best=None
    for tau in np.r_[0.05, np.arange(0.1, 1.6, 0.025)]:
        for dl in np.arange(0.0, 0.31, 0.05):
            yp = lpf(th, u_full, tau, dl)
            # gain via LS on moving part
            g = float(np.dot(yp[moving], y)/max(1e-9,np.dot(yp[moving],yp[moving])))
            e = np.sqrt(np.mean((g*yp[moving]-y)**2))
            if best is None or e<best[0]: best=(e,tau,dl,g)
    r['fit_all'] = {'rmse_deg': round(np.degrees(best[0]),3), 'tau': round(best[1],3), 'delay': round(best[2],3), 'gain': round(best[3],3)}
    # split fit: first 55% fit, last 45% validate
    n = len(th); cut = int(0.55*n); mv1 = moving.copy(); mv1[cut:]=False; mv2 = moving.copy(); mv2[:cut]=False
    best=None
    for tau in np.r_[0.05, np.arange(0.1, 1.6, 0.025)]:
        for dl in np.arange(0.0, 0.31, 0.05):
            yp = lpf(th, u_full, tau, dl); g = float(np.dot(yp[mv1], deff[mv1])/max(1e-9,np.dot(yp[mv1],yp[mv1])))
            e = np.sqrt(np.mean((g*yp[mv1]-deff[mv1])**2))
            if best is None or e<best[0]: best=(e,tau,dl,g)
    yp = lpf(th, u_full, best[1], best[2])*best[3]
    r['fit_first55'] = {'tau': round(best[1],3), 'delay': round(best[2],3), 'gain': round(best[3],3), 'fit_rmse_deg': round(np.degrees(best[0]),3), 'val_rmse_last45_deg': round(np.degrees(np.sqrt(np.mean((yp[mv2]-deff[mv2])**2))),3)}
    for tau_model in (0.05, 0.74):
        yp = lpf(th, u_full, tau_model, 0.0)
        r[f'model_tau{tau_model}_rmse_deg_all'] = round(np.degrees(np.sqrt(np.mean((yp[moving]-deff[moving])**2))),3)
        r[f'model_tau{tau_model}_rmse_deg_last45'] = round(np.degrees(np.sqrt(np.mean((yp[mv2]-deff[mv2])**2))),3)
    # cmd vs achieved: how often achieved and command have opposite sign (overshoot/lag indicator)
    cmd_on_truth = u_full
    r['frac_cmd_truth_opposite_sign(|cmd|>3deg)'] = round(float(np.mean((np.sign(cmd_on_truth)*np.sign(deff)<0)&(np.abs(cmd_on_truth)>np.radians(3))&moving)/max(1e-9,np.mean(moving))),4)
    # EKF vs truth speed
    lsel=(lp[:,0]>=t_on)&(lp[:,0]<=t_off); vl = np.hypot(lp[lsel,4],lp[lsel,5]); vt = np.interp(lp[lsel,0], th, v)
    r['ekf_minus_truth_speed_median_rmse'] = [round(float(np.median(vl-vt)),4), round(float(np.sqrt(np.mean((vl-vt)**2))),4)]
    out[name]=r
    np.savez(f'series_{name}.npz', tc=tc, steer_cmd=steer_cmd, thr=thr, th=th, yaw=yaw, yr=yr, v=v, deff=deff, ta=ta, sa=sa, lp=lp[lsel], hs=hs[hsel], ack=ack, cl=cl, mc=mc, hac=hac)
json.dump(out, open('resp_model_result.json','w'), indent=1, default=str)
for k,v in out.items():
    print('=====',k)
    for kk,vv in v.items(): print(f'  {kk}: {vv}')
