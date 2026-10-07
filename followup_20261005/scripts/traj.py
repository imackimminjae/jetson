import numpy as np, pickle, time, json
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
def ts(s): return time.mktime(time.strptime(s, '%Y-%m-%d %H:%M:%S'))
D = pickle.load(open('tlog_windows.pkl','rb'))
TARGET=(150.0,-140.0); TYAW=2.066348
align = {'A': dict(src=(-685.671021,-119.696877,1.300363), yd=0.765985, win=(ts('2026-10-03 05:59:08'), ts('2026-10-03 06:00:03'))),
         'B': dict(src=(-384.380493,114.051331,0.658912), yd=1.407436, win=(ts('2026-10-03 06:39:20'), ts('2026-10-03 06:40:09')))}
def wrap(a): return np.arctan2(np.sin(a),np.cos(a))
res={}
for k,al in align.items():
    a,b = al['win']
    lp = np.array([(d['_t'], d['x'], d['y'], d['vx'], d['vy']) for d in D['LOCAL_POSITION_NED'] if a<=d['_t']<=b])
    att = np.array([(d['_t'], d['yaw']) for d in D['ATTITUDE'] if a<=d['_t']<=b])
    hs = np.array([(d['_t'], d['lat']*1e-7, d['lon']*1e-7, d['vx']/100, d['vy']/100, d['yawspeed']) for d in D['HIL_STATE_QUATERNION'] if a<=d['_t']<=b])
    x_enu, y_enu = lp[:,2], lp[:,1]
    yaw_enu = wrap(np.pi/2 - np.interp(lp[:,0], att[:,0], np.unwrap(att[:,1])))
    sx,sy,syaw = al['src']; c,s = np.cos(al['yd']), np.sin(al['yd'])
    rx, ry = x_enu-sx, y_enu-sy
    mx = TARGET[0] + c*rx - s*ry; my = TARGET[1] + s*rx + c*ry; myaw = wrap(TYAW + yaw_enu - syaw)
    # check: first sample should be near source origin
    res[k] = dict(t=lp[:,0], x=mx, y=my, yaw=myaw, v_ekf=np.hypot(lp[:,3],lp[:,4]), first_enu=(x_enu[0],y_enu[0],yaw_enu[0]), hs=hs)
    print(k, 'first ENU sample', (round(x_enu[0],3), round(y_enu[0],3), round(yaw_enu[0],4)), 'logged source', al['src'], '-> map first', (round(mx[0],2), round(my[0],2)))
# validate A against bag odom
bag = np.load('bagA_odom.npz'); A=res['A']
ix = np.interp(bag['ot'], A['t'], A['x']); iy = np.interp(bag['ot'], A['t'], A['y'])
err = np.hypot(ix-bag['oxy'][:,0], iy-bag['oxy'][:,1])
print('A: reconstructed map pos vs bag odom_map: median err %.3f m, p95 %.3f m, max %.3f' % (np.median(err), np.quantile(err,.95), err.max()))
for lag in (-0.1,-0.05,0.05,0.1):
    ix = np.interp(bag['ot']+lag, A['t'], A['x']); iy = np.interp(bag['ot']+lag, A['t'], A['y']); e=np.hypot(ix-bag['oxy'][:,0], iy-bag['oxy'][:,1]); print('  lag',lag,'median',round(np.median(e),3))
# global path progress + lateral deviation proxy
GP = np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
seg = np.diff(GP,axis=0); seglen = np.hypot(*seg.T); cum = np.r_[0,np.cumsum(seglen)]
def project(px,py):
    best = None
    for i in range(len(seg)):
        d = seg[i]; v = np.stack([px-GP[i,0], py-GP[i,1]],-1); u = np.clip((v@d)/(d@d),0,1); q = GP[i]+u[:,None]*d
        dist = np.hypot(px-q[:,0], py-q[:,1]); sgn = np.sign(d[0]*(py-q[:,1]) - d[1]*(px-q[:,0]))
        prog = cum[i]+u*seglen[i]
        if best is None: best=[dist, prog, sgn*dist]
        else:
            m = dist<best[0]; best[0]=np.where(m,dist,best[0]); best[1]=np.where(m,prog,best[1]); best[2]=np.where(m,sgn*dist,best[2])
    return best
summary={}
fig,ax = plt.subplots(1,2,figsize=(16,9))
ax[0].plot(GP[:,0],GP[:,1],'k--o',label='global path (scenario1)')
for k,col in (('A','tab:red'),('B','tab:blue')):
    r=res[k]; dist,prog,sd = project(r['x'],r['y']); r['prog']=prog; r['lat']=sd
    moving = r['v_ekf']>1.0
    t0 = r['t'][moving][0]
    summary[k]=dict(start_clock=clock(t0), end_clock=clock(r['t'][moving][-1]), progress_m_at_end=float(prog[moving][-1]), progress_m_at_start=float(prog[moving][0]),
                    lat_dev_from_global_abs_p50_p95_max=[float(np.quantile(np.abs(sd[moving]),q)) for q in (.5,.95,1)],
                    pos_at_plus_sec={s: [float(np.interp(t0+s, r['t'], r['x'])), float(np.interp(t0+s, r['t'], r['y'])), float(np.interp(t0+s, r['t'], prog))] for s in (10,20,24,30,36)})
    ax[0].plot(r['x'][moving], r['y'][moving], color=col, label=f'run {k} track (EKF→map)')
    for s in range(0,40,5):
        tt=t0+s
        if tt<=r['t'][-1]: ax[0].annotate(f'{k}+{s}s', (np.interp(tt,r['t'],r['x']), np.interp(tt,r['t'],r['y'])), fontsize=7, color=col)
    ax[1].plot(r['t'][moving]-t0, sd[moving], color=col, label=f'run {k} signed dev from global polyline')
# A failures (bag) and B failure (log)
failA = {'05:59:42.712':(111.2,-29.29),'05:59:46.715':(120.86,-11.77),'05:59:47.714':(122.93,-7.5),'05:59:51.714':(123.87,9.13),'05:59:55.712':(134.52,23.27)}
for c_,(x,y) in failA.items(): ax[0].plot(x,y,'rx',ms=10)
tB = 1790977198.815; B=res['B']; bx,by = np.interp(tB,B['t'],B['x']), np.interp(tB,B['t'],B['y']); ax[0].plot(bx,by,'bx',ms=12,label='B QP fail 06:39:58.8')
summary['B_fail_pos']=[float(bx),float(by), float(np.interp(tB,B['t'],B['prog']))]
tBd = 1790977197.814; summary['B_interval_diag_pos']=[float(np.interp(tBd,B['t'],B['x'])), float(np.interp(tBd,B['t'],B['y']))]
ax[0].plot(bag['oxy'][:,0], bag['oxy'][:,1], 'r:', lw=1, label='run A bag odom_map (check)')
ax[0].set_aspect('equal'); ax[0].legend(fontsize=8); ax[0].grid(True); ax[0].set_title('Tracks in map frame (reconstructed from tlog LOCAL_POSITION_NED + logged alignment)')
ax[1].legend(); ax[1].grid(True); ax[1].set_xlabel('s since moving'); ax[1].set_ylabel('m (+left of global polyline)')
plt.tight_layout(); plt.savefig('tracks_AB.png', dpi=110)
json.dump(summary, open('traj_summary.json','w'), indent=1)
print(json.dumps(summary, indent=1))
pickle.dump({k:{kk:vv for kk,vv in v.items()} for k,v in res.items()}, open('traj_res.pkl','wb'))
