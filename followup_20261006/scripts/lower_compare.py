"""Lower-trace comparison A (05:59 bag) vs R3 (23:45 bag): tracking error, saturation, speed bias, plan change."""
import pickle, numpy as np, json
from analyze_runs import clock
SER=pickle.load(open('../runs_series.pkl','rb'))
BAGS={'A_1003_0559_tau005':'/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl',
      'R3_1005_2345_sc1':'/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis/decoded.pkl'}
out={}
for k,p in BAGS.items():
    d=pickle.load(open(p,'rb')); s=SER[k]
    tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); t=tr[:,0]; valid=tr[:,10]==1
    prog=np.interp(t,s['lp_t'],s['prog']); vt=np.interp(t,s['th'],s['v'])
    r={'trace_span':[clock(t[0]),clock(t[-1])],'progress_span_m':[round(float(prog[0]),1),round(float(prog[-1]),1)],'tau_in_trace':float(tr[0,23])}
    for seg,m in (('prog_50_88',(prog>=50)&(prog<88)),('prog_88_175',(prog>=88)&(prog<175))):
        mm=m&valid
        if mm.sum()<10: continue
        r[seg]=dict(n=int(mm.sum()),lat_err_p50_p95_max=[round(float(x),3) for x in np.quantile(np.abs(tr[mm,6]),[.5,.95,1])],
            head_err_p95_deg=round(float(np.degrees(np.quantile(np.abs(tr[mm,7]),.95))),2),
            sat_frac=round(float(np.mean(np.abs(tr[mm,8])>0.29845)),4),
            ctrl_speed_minus_truth_median=round(float(np.median(tr[mm,4]-vt[mm])),3),
            pred_eff_minus_applied_p95_deg=round(float(np.degrees(np.quantile(np.abs(tr[mm,19]-tr[mm,20]),.95))),2))
    r['modes']={str(int(a)):int(b) for a,b in zip(*np.unique(tr[:,12],return_counts=True))}
    out[k]=r
json.dump(out,open('../lower_compare.json','w'),indent=1); print(json.dumps(out,indent=1))
