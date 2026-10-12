import pickle,json,gzip,numpy as np
def stats(name,tt,windows):
    g=[];
    for s,e in windows:
        x=tt[(tt>=s)&(tt<e)]
        if len(x)>2: g.append(np.diff(x))
    g=np.concatenate(g);return f"{name}: median {np.median(g)*1000:.0f} ms, p99 {np.percentile(g,99)*1000:.0f} ms, max {g.max():.2f} s, >2x nominal {np.mean(g>2*np.median(g))*100:.1f}%"
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'));W=[(a['start'],a['end']) for a in A['att']]
ev=np.array([e['t_start'] for t,e in D['/debug/upper_branch_event']]);lo=np.array([v[0] for t,v in D['/debug/lower_mpc_trace']]);od=np.array([x[0] for x in D['/motive/vehicle/odom_map']])
tm=[e for t,e in D['/debug/upper_stage_timing']]
print('== 10-09 all60')
for n,tt in [('upper cycle start',ev),('lower trace stamp',lo),('odom receive',od)]: print('  ',stats(n,tt,W))
keys=[k for k in tm[0].keys()] if tm else [];print('   upper_stage_timing keys',keys[:12])
tot=[e.get('total_ms') or e.get('total') for e in tm if isinstance(e,dict)];tot=[x for x in tot if x is not None]
if tot: print('   upper total ms: median %.1f p99 %.1f max %.1f'%(np.median(tot),np.percentile(tot,99),max(tot)))
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/';T=json.load(gzip.open(H+'telemetry.json.gz','rt'));S=json.load(open(H+'summary.json'));W2=[(a['start'],a['end']) for a in S['attempts']]
rx=T['receive_times'];d=T['data']
ev2=np.array([v[0] for _,v in d['/debug/upper_interval_centering_state']]);lo2=np.array([v[0] for _,v in d['/debug/lower_mpc_trace']]);od2=np.array(rx['/motive/vehicle/odom_map'])
print('== 10-08 S3')
for n,tt in [('upper cycle start',ev2),('lower trace stamp',lo2),('odom receive',od2)]: print('  ',stats(n,tt,W2))
tm2=[v for _,v in d.get('/debug/upper_stage_timing',[])];tot2=[e.get('total_ms') or e.get('total') for e in tm2 if isinstance(e,dict)];tot2=[x for x in tot2 if x is not None]
if tot2: print('   upper total ms: median %.1f p99 %.1f max %.1f'%(np.median(tot2),np.percentile(tot2,99),max(tot2)))
