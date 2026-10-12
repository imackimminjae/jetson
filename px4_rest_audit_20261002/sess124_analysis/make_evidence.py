from pathlib import Path
import json,numpy as np
from pyulog import ULog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path(__file__).resolve().parent;u=ULog(str(p.parent/'sess124_log100.ulg'));d={x.name:x.data for x in u.data_list};t0=int(d['vehicle_local_position_groundtruth']['timestamp'][0]);ts=lambda q:(q['timestamp'].astype('int64')-t0)*1e-6
fig,ax=plt.subplots(4,1,figsize=(13,11),sharex=True)
for name,label,c in [('vehicle_local_position_groundtruth','SIH true speed','#087e8b'),('vehicle_local_position','EKF speed','#d95f02')]:
 q=d[name];ax[0].plot(ts(q),np.hypot(q['vx'],q['vy']),label=label,color=c,lw=2)
ax[0].axvspan(177.885339,193.600004,color='#e74c3c',alpha=.14,label='Moving >0.2 m/s, both rest flags TRUE')
ax[0].annotate('Rest asserted while true speed = 3.68 m/s',xy=(177.695397,3.68388),xytext=(183,5.9),arrowprops={'arrowstyle':'->'},fontsize=10)
ax[0].set_ylabel('Speed (m/s)');ax[0].set_ylim(-.15,6.8)
fields=[('vehicle_land_detected','at_rest','Land detector at_rest'),('estimator_status_flags','cs_vehicle_at_rest','EKF cs_vehicle_at_rest'),('estimator_status_flags','cs_gnss_pos','GNSS position fusion'),('vehicle_local_position','xy_valid','Horizontal position valid')]
for row,(name,key,label) in enumerate(fields):
 q=d[name];base=3-row;ax[1].step(ts(q),q[key]*.6+base,where='post',label=label,lw=1.8)
ax[1].set_yticks([.3,1.3,2.3,3.3],[v[2] for v in fields[::-1]],fontsize=9);ax[1].set_ylim(-.2,3.9)
q=d['estimator_gps_status'];t=ts(q)
for key,label,c in [('position_drift_rate_horizontal_m_s','Horizontal position drift','#b2182b'),('filtered_horizontal_speed_m_s','Filtered horizontal speed','#756bb1')]:ax[2].plot(t,q[key],label=label,color=c,lw=1.8)
ax[2].axhline(.1,color='black',ls='--',label='Check threshold: 0.1 m/s');ax[2].set_ylabel('GPS drift checks (m/s)')
q=d['vehicle_imu_status'];ax[3].plot(ts(q),q['gyro_vibration_metric'],label='Gyro vibration metric',color='#227a2a');ax[3].axhline(.02,color='black',ls='--',label='Motion threshold: 0.02');ax[3].set_ylim(0,.07);ax[3].set_ylabel('Gyro vibration metric')
ax[3].annotate('Last above-threshold sample: 176.690 s\nRest flag: 177.695 s (1.005 s later)',xy=(176.69006,.02220857),xytext=(182,.045),arrowprops={'arrowstyle':'->'},fontsize=9)
for a in ax:
 a.axvline(174.94,color='gray',ls=':',alpha=.65)
 a.axvline(194.9025,color='#087e8b',ls='--',alpha=.6)
 a.axvline(219.59535,color='#756bb1',ls='--',alpha=.6)
 a.grid(alpha=.2)
ax[0].legend(fontsize=8,loc='upper right');ax[2].legend(fontsize=8,loc='upper right');ax[3].legend(fontsize=8,loc='upper right')
ax[-1].set(xlim=(165,230),xlabel='Seconds from first recorded SIH groundtruth sample (log starts ~0.045 s earlier)')
fig.suptitle('Confirmed: the vehicle is classified at rest while coasting\nDashed vertical lines: actual stop (194.903 s), GNSS / horizontal validity recovery (219.595 s)',fontsize=13);fig.tight_layout();fig.savefig(p/'rest_mismatch_evidence.png',dpi=160)
truth=d['vehicle_local_position_groundtruth'];tt=ts(truth);speed=np.hypot(truth['vx'],truth['vy']);events=[(177.695397,'land.at_rest -> true'),(177.885339,'EKF.cs_vehicle_at_rest -> true'),(178.215437,'GPS horizontal position drift fails'),(178.595366,'GPS horizontal speed drift fails'),(183.095637,'xy_valid and v_xy_valid -> false'),(185.215671,'GNSS position and velocity fusion -> false'),(194.9025,'SIH exact speed -> 0'),(214.715557,'GPS horizontal speed failure clears'),(218.595491,'GPS horizontal position failure finally clears'),(219.59535,'GNSS and horizontal validity recover')]
rows=[]
for t,event in events:
 i=np.searchsorted(tt,t,side='right')-1;rows.append({'elapsed_s':t,'event':event,'truth_speed_m_s':float(speed[i])})
(p/'evidence_events.json').write_text(json.dumps(rows,indent=2))
