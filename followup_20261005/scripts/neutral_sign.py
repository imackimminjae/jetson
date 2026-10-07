import numpy as np, pickle, time
def clock(t): return time.strftime('%H:%M:%S', time.localtime(t)) + ('%.3f' % (t % 1))[1:]
d = pickle.load(open('/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'))
tr = np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); ttr = np.array([x['t'] for x in d['/debug/lower_mpc_trace']])
A = np.load('series_A_0559_tau0.05.npz')
m = (ttr>=A['tc'][0])&(ttr<=A['tc'][-1])
for col,name in ((13,'virtual steer norm'),(17,'mavlink_steering norm'),(16,'mavlink_throttle')):
    tl = np.interp(ttr, A['tc'], A['steer_cmd']/0.35 if col!=16 else A['thr'])
    for sgn in (1,-1):
        print(name,'sign',sgn,'rmse norm', round(float(np.sqrt(np.mean((sgn*tl[m]-tr[m,col])**2))),4))
# trace around the gap
i = np.argmax(np.diff(ttr)); print('trace gap', clock(ttr[i]), '->', clock(ttr[i+1]), round(ttr[i+1]-ttr[i],3), 'mode before/after', tr[i,12], tr[i+1,12], 'speed', tr[i,4])
# neutral stretches from tlog for both runs (throttle==0 and steer==0 while truth speed>1)
for name in ('A_0559_tau0.05','B_0639_tau0.74'):
    S = np.load(f'series_{name}.npz'); cl = S['cl']; cl = cl[cl[:,1]==187]
    v = np.interp(cl[:,0], S['th'], S['v'])
    neutral = (cl[:,2]==0)&(cl[:,3]==0)&(v>1.0)
    # group
    stretches=[]; start=None
    for i in range(len(cl)):
        if neutral[i] and start is None: start=cl[i,0]
        if (not neutral[i]) and start is not None: stretches.append((clock(start), round(cl[i-1,0]-start+0.0,3), round(float(v[i]),2))); start=None
    if start is not None: stretches.append((clock(start), round(cl[-1,0]-start,3), round(float(v[-1]),2)))
    print(name, 'neutral stretches (start, dur, speed):', stretches)
    # command vs HIL_ACTUATOR 1:1? count distinct steering values in each
    hac = S['hac']; print('  cmd187 n=%d, hil_act n=%d, ack n=%d' % (len(cl), len(hac), len(S['ack'])))
    # ack latency
    ack = S['ack']; ack = ack[ack[:,1]==187]
    lat = [ack[np.searchsorted(ack[:,0], t),0]-t for t in cl[:200,0] if np.searchsorted(ack[:,0], t)<len(ack)]
    print('  ack latency median/max (first 200 cmds): %.4f / %.4f s' % (np.median(lat), np.max(lat)))
