import pickle, numpy as np, time
d = pickle.load(open('/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'))
A = np.load('series_A_0559_tau0.05.npz')
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
print('bag topics:', sorted(d.keys()))
cmd = d['/control/applied_cmd']; tcmd = np.array([x['t'] for x in cmd]); steer_bag = np.array([x['angular'][2] for x in cmd]); thr_bag = np.array([x['linear'][0] for x in cmd])
print('applied_cmd sample', cmd[100])
tr = np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); ttr = np.array([x['t'] for x in d['/debug/lower_mpc_trace']])
print('trace ncols', tr.shape, 'col26 (steer cmd rad) sample', tr[100,26], 'col31 scale', tr[100,31])
# compare tlog cmd (rad) vs trace col26 at trace times
tl = np.interp(ttr, A['tc'], A['steer_cmd'])
m = (ttr>=A['tc'][0])&(ttr<=A['tc'][-1])
for lag in (-0.1,-0.05,0,0.05,0.1):
    tl2 = np.interp(ttr+lag, A['tc'], A['steer_cmd']); print('lag',lag,'rmse deg tlog vs trace col26', np.degrees(np.sqrt(np.mean((tl2[m]-tr[m,26])**2))))
print('trace col26 vs applied_cmd angular.z: ', np.degrees(np.sqrt(np.mean((np.interp(ttr, tcmd, steer_bag)[m]-tr[m,26])**2))))
# trace timestamps gaps
dt = np.diff(ttr); print('trace gaps >0.15:', [(clock(ttr[i]), round(g,3)) for i,g in enumerate(dt) if g>0.15])
# lateral error column? print a header guess: first 12 cols of a sample
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print('trace sample row', tr[200])
# odom_map positions for run A (for alignment check)
od = d['/motive/vehicle/odom_map']; ot = np.array([x['t'] for x in od]); oxy = np.array([x['xy'] for x in od]); oyaw = np.array([x['q'][2] for x in od]); ov = np.array([x['v'] for x in od])
np.savez('bagA_odom.npz', ot=ot, oxy=oxy, oyaw=oyaw, ov=ov)
print('odom_map first/last', clock(ot[0]), oxy[0], clock(ot[-1]), oxy[-1])
gp = d['/navigation/global_path'][0]['xy']; print('global path', gp)
sp = d['/planner/upper_path_sparse']; print('sparse paths', len(sp), 'first stamp', clock(sp[0]['t']))
