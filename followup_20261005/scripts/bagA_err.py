import numpy as np, pickle, time
d = pickle.load(open('/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'))
res = pickle.load(open('traj_res.pkl','rb')); R=res['A']
tr = np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); ttr=np.array([x['t'] for x in d['/debug/lower_mpc_trace']])
S=np.load('series_A_0559_tau0.05.npz')
prog = np.interp(tr[:,0], R['t'], R['prog']); valid = tr[:,10]==1
vt = np.interp(tr[:,0], S['th'], S['v'])
for name,m in (('common_0_88m',(prog<88)&valid),('after_88m',(prog>=88)&valid),('all_valid',valid)):
    print(name, 'n',m.sum(), 'lat_err p50/p95/max %.3f/%.3f/%.3f' % tuple(np.quantile(np.abs(tr[m,6]),[.5,.95,1])), 'heading_err_deg p95 %.2f' % np.degrees(np.quantile(np.abs(tr[m,7]),.95)),
          'ctrl speed - truth: median %.3f rmse %.3f' % (np.median(tr[m,4]-vt[m]), np.sqrt(np.mean((tr[m,4]-vt[m])**2))))
# odom_map twist speed vs truth
od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); ov=np.array([np.hypot(*x['v']) for x in od]); vt2=np.interp(ot,S['th'],S['v']); m=(ot>=S['tc'][0])&(ot<=S['tc'][-1])
print('odom_map twist speed - truth: median %.3f rmse %.3f' % (np.median(ov[m]-vt2[m]), np.sqrt(np.mean((ov[m]-vt2[m])**2))))
# receipt-interval jitter of odom_map and of tlog LOCAL_POSITION_NED
dto=np.diff(ot[m]); print('odom_map receipt dt: median %.4f std %.4f min %.4f max %.4f' % (np.median(dto), dto.std(), dto.min(), dto.max()))
lp=S['lp']; dtl=np.diff(lp[:,0]); print('tlog LOCAL_POSITION_NED dt: median %.4f std %.4f min %.4f max %.4f' % (np.median(dtl), dtl.std(), dtl.min(), dtl.max()))
