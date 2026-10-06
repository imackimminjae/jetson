import json,pickle,numpy as np
from pathlib import Path
from datetime import datetime
r=Path('/home/imac/ros2_ws/drive_debug_20260923_232054_light/analysis');new=json.load((r/'requested_changes_replay.json').open());old=pickle.load((r/'decoded.pkl').open('rb'));odom=old['/motive/vehicle/odom_map'];xy=np.array([p['xy'] for p in odom]);time=lambda t:datetime.fromtimestamp(t-new['wall_to_bag_offset']).strftime('%H:%M:%S.%f')[:-3]
errors=[];failed=[]
for e in new['events']:
 p=odom[np.argmin(np.sum((xy-[e['x'],e['y']])**2,axis=1))];v=np.linalg.norm(p['v']);w=p['omega'];maximum=min(.7,.5*5.4*np.tan(.3)/2.8);center=np.clip(.5*5.4*(np.clip(w/v,-np.tan(.3)/2.8,np.tan(.3)/2.8) if v>.1 else 0),-maximum,maximum);lo=max(-maximum,center-.0872664626);hi=min(maximum,center+.0872664626)
 if e['valid_plan']:
  delta=np.subtract(e['planned_world'][1],e['planned_world'][0]);angle=np.arctan2(delta[1],delta[0])-e['yaw_rad'];angle=np.arctan2(np.sin(angle),np.cos(angle));errors.append(max(lo-angle,angle-hi,0))
 else:failed.append(e);print('rejected',time(e['t_start']),e['status'])
print('max published first heading bound violation rad',max(errors));assert max(errors)<.001
lower=np.array(new['lower']);valid=lower[:,10]==1
print('valid lower',int(sum(valid)),'max steer',max(abs(lower[valid,9])),'max internal solver time ms',max(lower[valid,24]));assert np.all(abs(lower[valid,9])<=1.00001)
mode3=lower[lower[:,12]==3];tfirst=new['events'][0]['t_emit'];initial=mode3[:,0]<tfirst;print('neutral samples before first plan/after',int(sum(initial)),int(sum(~initial)))
for x in mode3[~initial][::5]:print('neutral',time(x[0]))
events=new['events'];ok=[e for e in events if e['valid_plan']];gaps=np.diff([e['t_emit'] for e in ok]);print('max successful path interval sec',max(gaps))
