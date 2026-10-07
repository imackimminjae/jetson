"""Unified tlog-based run comparison (steering response, saturation, neutral, track, progress).
Runs A/B: 2026-10-03 (data from followup_20261005/data_20261003/tlog_windows.pkl)
Runs R1/R2/R3: 2026-10-05 (tlog_20261005.pkl). Alignment values come from px4_odom_map_bridge ROS logs."""
import pickle, json, time, numpy as np
def ts(s): return time.mktime(time.strptime(s, '%Y-%m-%d %H:%M:%S'))
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
def wrap(a): return np.arctan2(np.sin(a), np.cos(a))
L, LR, SCALE = 2.8, 1.4, 0.35
ROUTES = {'scenario1': np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float),
          'scenario3': np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)}
OLD = '/home/imac/ros2_ws/followup_20261005/data_20261003/tlog_windows.pkl'
NEW = '/home/imac/ros2_ws/followup_20261006/tlog_20261005.pkl'
RUNS = {
 'A_1003_0559_tau005': dict(pkl=OLD, win=('2026-10-03 05:59:10','2026-10-03 06:00:03'), route='scenario1', target=(150,-140,2.066348), src=(-685.671021,-119.696877,1.300363), yd=0.765985),
 'B_1003_0639_tau074': dict(pkl=OLD, win=('2026-10-03 06:39:15','2026-10-03 06:40:09'), route='scenario1', target=(150,-140,2.066348), src=(-384.380493,114.051331,0.658912), yd=1.407436),
 'R1_1005_2341_sc3':   dict(pkl=NEW, win=('2026-10-05 23:41:05','2026-10-05 23:43:24'), route='scenario3', target=(187,220,-1.570796), src=(0.147999,0.039396,1.700082), yd=3.012307),
 'R2_1005_2344_sc1':   dict(pkl=NEW, win=('2026-10-05 23:44:10','2026-10-05 23:45:00'), route='scenario1', target=(150,-140,2.066348), src=(-3.454166,107.481651,0.958669), yd=1.107679),
 'R3_1005_2345_sc1':   dict(pkl=NEW, win=('2026-10-05 23:45:05','2026-10-05 23:45:53'), route='scenario1', target=(150,-140,2.066348), src=(191.577255,192.487167,-0.083638), yd=2.149986),
}
def q2yaw(q):
    w,x,y,z = q; return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def eff_steer(yr, v):
    v = np.maximum(v, 0.3); d = np.arctan2(L*yr, v)
    for _ in range(10): d = np.arctan(L*yr/(v*np.cos(np.arctan(LR*np.tan(d)/L))))
    return d
def lpf(t,u,tau):
    y=np.zeros_like(u)
    for i in range(1,len(t)): y[i]=y[i-1]+(1-np.exp(-(t[i]-t[i-1])/tau))*(u[i]-y[i-1])
    return y
def project(GP, px, py):
    seg=np.diff(GP,axis=0); sl=np.hypot(*seg.T); cum=np.r_[0,np.cumsum(sl)]; best=None
    for i in range(len(seg)):
        d=seg[i]; v=np.stack([px-GP[i,0],py-GP[i,1]],-1); u=np.clip((v@d)/(d@d),0,1); q=GP[i]+u[:,None]*d
        dist=np.hypot(px-q[:,0],py-q[:,1]); sd=np.sign(d[0]*(py-q[:,1])-d[1]*(px-q[:,0]))*dist; pr=cum[i]+u*sl[i]
        if best is None: best=[dist,pr,sd]
        else:
            m=dist<best[0]; best=[np.where(m,dist,best[0]),np.where(m,pr,best[1]),np.where(m,sd,best[2])]
    return best[1], best[2]
cache={}
def load(p):
    if p not in cache: cache[p]=pickle.load(open(p,'rb'))
    return cache[p]
def arr(D,typ,a,b,f): return np.array([f(d) for d in D[typ] if a<=d['_t']<=b])
def main():
    OUT={}; SER={}
    for name,cfg in RUNS.items():
        D=load(cfg['pkl']); a,b=ts(cfg['win'][0]),ts(cfg['win'][1])
        cl=arr(D,'COMMAND_LONG',a,b,lambda d:(d['_t'],d['command'],d['param1'],d['param2'])); cl=cl[cl[:,1]==187]
        hs=arr(D,'HIL_STATE_QUATERNION',a,b,lambda d:(d['_t'],q2yaw(d['attitude_quaternion']),d['yawspeed'],d['vx']/100,d['vy']/100))
        lp=arr(D,'LOCAL_POSITION_NED',a,b,lambda d:(d['_t'],d['time_boot_ms']*1e-3,d['x'],d['y'],d['vx'],d['vy']))
        att=arr(D,'ATTITUDE',a,b,lambda d:(d['_t'],d['yaw']))
        hac=arr(D,'HIL_ACTUATOR_CONTROLS',a,b,lambda d:(d['_t'],d['controls'][0],d['controls'][1]))
        tc=cl[:,0]; sc=cl[:,3]*SCALE; thr=cl[:,2]
        th=hs[:,0]; yr=hs[:,2]; v=np.hypot(hs[:,3],hs[:,4]); deff=eff_steer(yr,v)
        # track
        xe,ye=lp[:,3],lp[:,2]; yawe=wrap(np.pi/2-np.interp(lp[:,0],att[:,0],np.unwrap(att[:,1])))
        sx,sy,syaw=cfg['src']; c,s=np.cos(cfg['yd']),np.sin(cfg['yd']); tx,ty,tyaw=cfg['target']
        mx=tx+c*(xe-sx)-s*(ye-sy); my=ty+s*(xe-sx)+c*(ye-sy)
        GP=ROUTES[cfg['route']]; prog,lat=project(GP,mx,my)
        moving_lp=np.hypot(lp[:,4],lp[:,5])>0.5
        # restrict progress to after start (monotone-ish)
        act=np.where(thr>0)[0]
        r=dict(route=cfg['route'], window=[clock(a),clock(b)])
        if len(act)==0: OUT[name]=r; continue
        t_on,t_off=tc[act[0]],tc[act[-1]]
        r['throttle_active']=[clock(t_on),clock(t_off)]; r['active_s']=round(t_off-t_on,1)
        r['map_first_xy']=[round(mx[0],2),round(my[0],2)]
        mv=(th>=t_on)&(th<=t_off)&(v>1.0)
        u=np.interp(th,tc,sc,left=0.0)
        mc=(tc>=t_on)&(tc<=t_off)
        progh=np.interp(th,lp[:,0],prog); lath=np.interp(th,lp[:,0],lat); progc=np.interp(tc,lp[:,0],prog)
        def stats(mh,mcc):
            o={}
            if mh.sum()<20: return None
            o['dur_s']=round(float(np.sum(np.diff(th)[mh[1:]])),1); o['truth_speed_p50']=round(float(np.median(v[mh])),2)
            o['cmd_abs_p95_deg']=round(float(np.degrees(np.quantile(np.abs(sc[mcc]),.95))),2)
            o['cmd_sat_frac']=round(float(np.mean(np.abs(sc[mcc])>=0.298)),4)
            o['eff_abs_p95_deg']=round(float(np.degrees(np.quantile(np.abs(deff[mh]),.95))),2)
            o['yawrate_abs_p95_dps']=round(float(np.degrees(np.quantile(np.abs(yr[mh]),.95))),2)
            o['opp_sign_frac']=round(float(np.mean((np.sign(u[mh])*np.sign(deff[mh])<0)&(np.abs(u[mh])>np.radians(3)))),4)
            o['lat_dev_global_p50_p95_m']=[round(float(np.quantile(np.abs(lath[mh]),q)),2) for q in (.5,.95)]
            for tau in (0.05,0.74): o[f'rmse_model_tau{tau}_deg']=round(float(np.degrees(np.sqrt(np.mean((lpf(th,u,tau)[mh]-deff[mh])**2)))),3)
            best=None
            for tau in np.arange(0.1,1.6,0.025):
                yp=lpf(th,u,tau); g=float(np.dot(yp[mh],deff[mh])/max(1e-12,np.dot(yp[mh],yp[mh]))); e=np.sqrt(np.mean((g*yp[mh]-deff[mh])**2))
                if best is None or e<best[0]: best=(e,tau,g)
            o['fit_tau_gain_rmse']=[round(best[1],3),round(best[2],3),round(float(np.degrees(best[0])),3)]
            scu=sc[mcc]; keep=np.r_[True,np.abs(np.diff(scu))>1e-6]; scu=scu[keep]
            o['cmd_reversals_per_s']=round(float(np.sum(np.sign(scu[1:])*np.sign(scu[:-1])<0)/max(1e-9,o['dur_s'])),3)
            return o
        r['all']=stats(mv,mc)
        if cfg['route']=='scenario1':
            r['common_0_88m']=stats(mv&(progh<88),mc&(progc<88)); r['after_88m']=stats(mv&(progh>=88),mc&(progc>=88))
            r['progress_end_m']=round(float(prog[-1]),1)
        # gaps & neutral
        dtc=np.diff(tc[mc]); r['cmd_rate_hz']=round(1/np.median(dtc),2); r['cmd_max_gap_s']=round(float(dtc.max()),3)
        r['cmd_gaps_gt_0.3s']=[(clock(tc[mc][i]),round(g,3)) for i,g in enumerate(dtc) if g>0.3]
        vc=np.interp(tc,th,v); neu=(thr==0)&(cl[:,3]==0)&(vc>1.0)&mc
        st=[];s0=None
        for i in range(len(tc)):
            if neu[i] and s0 is None: s0=i
            if not neu[i] and s0 is not None: st.append((clock(tc[s0]),round(tc[i-1]-tc[s0],2),round(float(vc[s0]),2))); s0=None
        r['neutral_stretches(start,dur,speed)']=st
        lsel=(lp[:,0]>=t_on)&(lp[:,0]<=t_off); vt=np.interp(lp[lsel,0],th,v); vl=np.hypot(lp[lsel,4],lp[lsel,5])
        r['ekf_minus_truth_speed_median_rmse']=[round(float(np.median(vl-vt)),3),round(float(np.sqrt(np.mean((vl-vt)**2))),3)]
        OUT[name]=r
        SER[name]=dict(tc=tc,sc=sc,thr=thr,th=th,yr=yr,v=v,deff=deff,lp_t=lp[:,0],mx=mx,my=my,prog=prog,lat=lat,t_on=t_on,t_off=t_off,route=cfg['route'],lp=lp)
    json.dump(OUT,open('../runs_summary.json','w'),indent=1)
    pickle.dump(SER,open('../runs_series.pkl','wb'))
    for k,v in OUT.items():
        print('=====',k)
        for kk,vv in v.items(): print('  ',kk,':',vv)

if __name__=='__main__':
    main()
