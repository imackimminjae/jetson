from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime
from zoneinfo import ZoneInfo
from matplotlib.dates import date2num,DateFormatter
r=Path(__file__).resolve().parent;d=np.load(r/'comparison.npz');f=np.load(r/'fast_response.npz');tz=ZoneInfo('Asia/Seoul');xx=lambda t:[date2num(datetime.fromtimestamp(float(v),tz)) for v in t]
fig,ax=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True);m=d['active'];t=d['t'][m];ax[0].plot(xx(t),np.degrees(d['beta'][m]),label='Reported center-point course - yaw');h=np.load(r/'telemetry.npz')['truth'];rear_lateral=d['lateral']-1.4*h[:,3];rear_forward=d['speed']*np.cos(d['beta']);ax[0].plot(xx(t),np.degrees(np.arctan2(rear_lateral,rear_forward))[m],label='After 1.4 m rear-axle offset');ax[0].set_ylabel('Angle (deg)');ax[0].legend(fontsize=8)
ax[1].plot(xx(d['model_t']),np.degrees(d['truth_rate']),color='black',label='SIH actual');ax[1].plot(xx(d['model_t']),np.degrees(d['model_rate']),label='Existing MPC response');ax[1].plot(xx(f['t']),np.degrees(f['pred']),ls='--',label='Fast CG kinematic response');ax[1].set_ylabel('Yaw rate (deg/s)');ax[1].legend(fontsize=8)
a=np.load(r/'telemetry.npz')['att'];ay=np.interp(d['t'],a[:,0],np.unwrap(a[:,3]));err=np.arctan2(np.sin(ay-d['yaw']),np.cos(ay-d['yaw']));ax[2].plot(xx(t),np.degrees(err[m]),color='tab:red');ax[2].set_ylabel('EKF - SIH yaw (deg)');ax[2].xaxis.set_major_formatter(DateFormatter('%H:%M:%S',tz=tz));ax[2].set_xlabel('2026-09-24 KST')
for a in ax:a.grid(alpha=.25)
ax[0].set_title('After bicycle-model change: reference point and response mismatch')
fig.savefig(r/'kinematic_comparison.png',dpi=150)
