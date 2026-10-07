import pickle, numpy as np, json
from analyze_runs import clock
SER=pickle.load(open('../runs_series.pkl','rb'))
BAG={'A_1003_0559_tau005':'/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis','R3_1005_2345_sc1':'/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis'}
out={}
for k,p in BAG.items():
    d=pickle.load(open(p+'/decoded.pkl','rb')); s=SER[k]
    tr=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]); t=tr[:,0]; valid=tr[:,10]==1
    truth=np.interp(t,s['th'],s['deff']); prog=np.interp(t,s['lp_t'],s['prog'])
    est=-tr[:,19]  # ROS sign -> MAVLink/NED sign
    r={}
    for seg,m in (('prog_50_88',(prog>=50)&(prog<88)&valid),('prog_88_175',(prog>=88)&(prog<175)&valid)):
        r[seg]={'internal_effective_steer_minus_truth_rmse_deg':round(float(np.degrees(np.sqrt(np.mean((est[m]-truth[m])**2)))),3)}
    f=json.load(open(p+'/features.json'))
    jp=np.array([(x['t'],abs(x.get('h1_jump_deg',0))) for x in f if x['valid']])
    pj=np.interp(jp[:,0],s['lp_t'],s['prog'])
    for seg,m in (('prog_50_88',(pj>=50)&(pj<88)),('prog_88_175',(pj>=88)&(pj<175))):
        r[seg]['plan_h1_jump_abs_p50_max_deg']=[round(float(np.median(jp[m,1])),1),round(float(jp[m,1].max()),1)] if m.sum() else None
    out[k]=r
print(json.dumps(out,indent=1)); json.dump(out,open('../model_state_and_jumps.json','w'),indent=1)
