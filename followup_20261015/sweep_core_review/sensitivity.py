from pathlib import Path
import pickle, numpy as np,json
from scipy.ndimage import uniform_filter1d
from scipy.signal import savgol_filter
P=Path(__file__).resolve().parent;C=pickle.loads((P/'curves.pkl').read_bytes());result=[]
for key,cc in C.items():
 aa=[]
 for c in cc:
  l=c['lower'];t=l[:,0];s=c['lower_s'];steer=np.degrees(-l[:,17]*l[:,31]);dt=np.diff(t);moving=l[:,4]>1
  cutoff=moving&(s>=15)&(s<=175)
  pairs=cutoff[:-1]&cutoff[1:]&(dt>.015)&(dt<.25)
  dist=np.sum(.5*(l[:-1,4]+l[1:,4])[pairs]*dt[pairs]);tv=np.sum(abs(np.diff(steer)[pairs]))/dist*100
  # Alternative includes command changes across the long trace gaps.
  pairsall=moving[:-1]&moving[1:]&(dt>.015)
  distall=np.sum(.5*(l[:-1,4]+l[1:,4])[pairsall]*dt[pairsall]);tvall=np.sum(abs(np.diff(steer)[pairsall]))/distall*100
  od=c['odom'];tt=np.arange(od[0,0],od[-1,0],.05);yr=np.degrees(savgol_filter(np.interp(tt,od[:,0],np.unwrap(od[:,3])),9,2,deriv=1,delta=.05));ss=np.interp(tt,od[:,0],c['s']);vv=np.interp(tt,od[:,0],np.hypot(od[:,5],od[:,6]));mask=(ss>=15)&(ss<=175)&(vv>1);ma=mask[1:]&mask[:-1];acc=np.diff(yr)/.05
  aa.append(dict(attempt=c['attempt'],outcome=c['outcome'],steering_tv_15_175_deg_per_100m=float(tv),steering_tv_all_gaps_deg_per_100m=float(tvall),yaw_accel_15_175_rms_degps2=float(np.sqrt(np.mean(acc[ma]**2)))))
 result.append(dict(id=key,attempts=aa))
(P/'sensitivity.json').write_text(json.dumps(result,indent=2)+'\n')
for r in result:print(r['id'], '15..175 TV/Yaw:',round(np.median([a['steering_tv_15_175_deg_per_100m'] for a in r['attempts']]),1),round(np.median([a['yaw_accel_15_175_rms_degps2'] for a in r['attempts']]),2),'include gap TV',round(np.median([a['steering_tv_all_gaps_deg_per_100m'] for a in r['attempts']]),1))
