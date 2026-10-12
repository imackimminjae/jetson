from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,gzip,numpy as np
H=Path(__file__).resolve().parent
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
d=json.loads(gzip.decompress((H/'telemetry.json.gz').read_bytes()));od=np.array(d['odom']);lo=np.array(d['lower']);out=[]
for a in json.loads((H/'attempts.json').read_text()):
 cs=json.load(open(a['input']))['cycles'];start,end=a['start'],a['end'];cs=[c for c in cs if c['running']];lv=lo[(lo[:,0]>=start)&(lo[:,0]<end)];valid=[c for c in cs if c['rec_valid']];invalid=[c for c in cs if not c['rec_valid']];trans=[];prev=None
 for z in lv:
  mode=int(z[12])
  if mode!=prev:trans.append(dict(time=clock(z[0]),t=z[0],mode=mode,xy=z[1:3].tolist(),speed=z[4]));prev=mode
 gaps={}
 for topic,tt in d['receive_times'].items():
  tt=np.array(tt);tt=tt[(tt>=start)&(tt<end)]
  if len(tt)>1:
   dt=np.diff(tt);j=int(np.argmax(dt));gaps[topic]=dict(max_sec=float(dt[j]),from_time=clock(tt[j]),to_time=clock(tt[j+1]))
 row=dict(attempt=a['attempt'],start=clock(start),end=clock(end),valid=len(valid),invalid=len(invalid),first_invalid={k:invalid[0][k] for k in ['t','x','y','rec_reason','rec_status','grid_age','grid_free','grid_near_free']} if invalid else None,last_valid={k:valid[-1][k] for k in ['t','x','y','rec_published']} if valid else None,lower_transitions=trans,gaps=gaps,events=[dict(time=clock(c['t']),xy=[c['x'],c['y']],valid=c['rec_valid'],N=len(c['rec_preview'])-1,B=c['rec_B'],grid_free=c['grid_free'],grid_near_free=c['grid_near_free'],end=c['rec_published'][-1:]) for c in cs if c['x']>-100])
 out.append(row);print(a['attempt'],'first bad',row['first_invalid'],'lower',trans[:8]);print('gaps',gaps.get('/bev/occupancy_grid'),gaps.get('/knu_batch/status'))
(H/'timeline.json').write_text(json.dumps(out,indent=2))
# Recording continued through abort. This cannot observe unrecorded supervisor heartbeats.
ab=1791447918.355
print('abort receive gaps')
for n,tt in d['receive_times'].items():
 tt=np.array(tt);tt=tt[(tt>ab-4)&(tt<ab+4)]
 if len(tt)>1:print(n,len(tt),'maxgap',np.diff(tt).max())
