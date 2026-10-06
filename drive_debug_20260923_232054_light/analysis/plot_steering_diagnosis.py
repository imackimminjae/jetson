from pathlib import Path
import pickle,json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/home/imac/ros2_ws/drive_debug_20260923_232054_light/analysis');d=pickle.load((root/'decoded.pkl').open('rb'));l=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]);truth=np.load(root/'sway_truth.npz')['truth'];g=np.load(root/'sway_geometry.npz')['curvature'];replay=json.load((root/'sway_replay.json').open());origin=1790173260;mask=(l[:,0]>=1790173287)&(l[:,0]<=1790173306);x=l[mask];t=x[:,0]-origin;ref=np.unwrap(x[:,3]-x[:,7]);psi=np.unwrap(x[:,3]);omega=-np.interp(x[:,0],truth[:,0],truth[:,1]);pred=x[:,4]/2.8*x[:,19]
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained');fig.suptitle('Steering oscillation | changing reference + sharp curvature demands',fontsize=15)
a=ax[0,0];a.plot(t,np.degrees(psi),label='Vehicle heading');a.plot(t,np.degrees(ref),label='Local reference heading');a.set(ylabel='Heading [deg]',title='Reference direction changes abruptly');a.legend(fontsize=8)
a=ax[0,1];a.plot(t,x[:,9],label='Recorded command');r=[v for v in replay if v['refresh']];a.scatter([v['t']-origin for v in r],[v['old_path_command'] for v in r],marker='x',color='tab:orange',label='Same state, previous path',zorder=4);a.set(ylabel='Normalized steering',ylim=(-1.15,1.15),title='Changing only the path changes the MPC command');a.legend(fontsize=8)
a=ax[1,0];gm=(g[:,0]>=1790173287)&(g[:,0]<=1790173306);a.plot(g[gm,0]-origin,g[gm,1],label='Maximum |curvature| in MPC preview');a.axhline(np.tan(.3)/2.8,color='tab:red',ls='--',label='Limit from steering 0.30 rad / wheelbase 2.8 m');a.set(ylabel='Curvature [1/m]',title='Polyline curvature exceeds configured steering capability');a.legend(fontsize=8)
a=ax[1,1];a.plot(t,np.degrees(omega),label='SIH measured yaw rate');a.plot(t,np.degrees(pred),ls='--',label='Current model estimate');a.set(ylabel='Yaw rate [deg/s]',title='Current response model follows actual rotation');a.legend(fontsize=8)
for a in ax.flat:a.set(xlabel='Seconds after 23:21:00',xlim=(27,46));a.grid(alpha=.2)
fig.savefig(root/'steering_diagnosis.png',dpi=150)
