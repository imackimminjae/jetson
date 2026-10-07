# Speed-bias mechanism check (run A bag + tlog). See followup_20261005/RESULT.txt section 5.
import numpy as np, pickle, time
d = pickle.load(open('/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'))
S=np.load('series_A_0559_tau0.05.npz')
tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); valid=tr[:,10]==1
od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); ov=np.hypot(*np.array([x['v'] for x in od]).T)
hs=S['hs']; th=hs[:,0]; vt=np.hypot(hs[:,4],hs[:,5])
m=valid&(tr[:,0]>=th[0])&(tr[:,0]<=th[-1])&(tr[:,0]>=ot[0]); idx=np.searchsorted(ot,tr[m,0],side='right')-1
print('trace speed - odom twist (sample&hold): median %.3f' % np.median(tr[m,4]-ov[idx]))
print('odom twist at lower sample instants - truth: median %.3f' % np.median(ov[idx]-np.interp(tr[m,0],th,vt)))
mm=(ot>=th[0])&(ot<=th[-1]); print('odom twist all samples - truth: median %.3f rmse %.3f' % (np.median(ov[mm]-np.interp(ot[mm],th,vt)), np.sqrt(np.mean((ov[mm]-np.interp(ot[mm],th,vt))**2))))
lp=S['lp']; dpl=np.hypot(*np.diff(lp[:,2:4],axis=0).T); dtr=np.diff(lp[:,0]); dtb=np.diff(lp[:,1])
a=time.mktime(time.strptime('2026-10-03 05:59:30','%Y-%m-%d %H:%M:%S')); j0=np.searchsorted(lp[:,0],a)
np.set_printoptions(precision=3,suppress=True,linewidth=200)
print('LOCAL_POSITION_NED dt(time_boot_ms):', dtb[j0:j0+12]); print('|dp|/dt_receipt:', (dpl/dtr)[j0:j0+12]); print('|dp|/dt_boot   :', (dpl/dtb)[j0:j0+12])
