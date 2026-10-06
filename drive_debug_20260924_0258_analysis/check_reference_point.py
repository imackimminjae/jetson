from pathlib import Path
import numpy as np,json
r=Path(__file__).resolve().parent;h=np.load(r/'telemetry.npz')['truth'];w,x,y,z=h[:,7:11].T;yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));vx=np.cos(yaw)*h[:,4]+np.sin(yaw)*h[:,5];vy=-np.sin(yaw)*h[:,4]+np.cos(yaw)*h[:,5];rate=h[:,3];speed=np.hypot(vx,vy);mask=(h[:,0]>1790186294)&(h[:,0]<1790186362.954)&(speed>1);turn=mask&(abs(rate)>.05)
lr=np.dot(rate[turn],vy[turn])/np.dot(rate[turn],rate[turn]);res=vy-1.4*rate;beta=np.arctan2(res,vx)
print('rear axle to reported point estimate metres',lr)
print('vy / yawrate median p05 p95',np.quantile(vy[turn]/rate[turn],[.5,.05,.95]))
print('assuming point offset1.4m rear axle lateral residual p50 p95 max mps',np.quantile(abs(res[mask]),[.5,.95,1]))
print('rear velocity vsbody heading difference p50 p95 max deg',np.degrees(np.quantile(abs(beta[mask]),[.5,.95,1])))
# Kinematic CG model: delta=atan(L*r/v_forward); beta=atan(lr/L*tan(delta)).
delta=np.arctan(2.8*rate/np.maximum(vx,.1));print('inferred steer maxdeg',np.degrees(max(abs(delta[mask]))))
(r/'reference_point.json').write_text(json.dumps({'rear_offset_fit_m':lr,'rear_lateral_residual_abs_quantiles_mps':np.quantile(abs(res[mask]),[.5,.95,1]).tolist(),'rear_beta_abs_quantiles_deg':np.degrees(np.quantile(abs(beta[mask]),[.5,.95,1])) .tolist()},indent=2))
