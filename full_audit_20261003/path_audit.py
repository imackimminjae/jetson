import json,pickle,numpy as np
from pathlib import Path
p=Path(__file__).resolve().parent;d=pickle.load(open(p.parent/'drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'));rows=[];prev=None;kmax=np.cos(np.arctan(.5*np.tan(.3)))*np.tan(.3)/2.8
for r in d['/planner/lower_reference_path/with_arclength']:
 xy=r['xy'];s=r['s'];psi=np.r_[np.unwrap(np.arctan2(np.diff(xy,axis=0)[:,1],np.diff(xy,axis=0)[:,0])),0.];psi[-1]=psi[-2];i=np.arange(len(s));left=np.maximum(i-1,0);right=np.minimum(i+1,len(s)-1);curv=(psi[right]-psi[left])/(s[right]-s[left]);row={'t':r['t'],'max_abs_polyline_curvature':float(max(abs(curv))),'sample_curvature_over_limit_fraction':float(np.mean(abs(curv)>kmax))}
 if prev is not None:
  old=prev['xy'];os=prev['s'];diff=np.diff(old,axis=0);v=xy[0]-old[:-1];u=np.clip(np.sum(v*diff,axis=1)/np.sum(diff*diff,axis=1),0,1);q=old[:-1]+u[:,None]*diff;j=np.argmin(np.linalg.norm(q-xy[0],axis=1));progress=os[j]+u[j]*(os[j+1]-os[j]);ahead=progress+4.5
  if ahead<os[-1] and s[-1]>4.5:
   oldpoint=np.array([np.interp(ahead,os,old[:,k]) for k in [0,1]]);newpoint=np.array([np.interp(4.5,s,xy[:,k]) for k in [0,1]]);row['matched_progress_4p5m_shift']=float(np.linalg.norm(newpoint-oldpoint))
 rows.append(row);prev=r
summary={'center_bicycle_curvature_limit_1pm':float(kmax),'plans':len(rows),'plans_with_curvature_spike_above_limit':int(sum(x['max_abs_polyline_curvature']>kmax for x in rows)),'curvature_max':max(x['max_abs_polyline_curvature'] for x in rows),'p95_matched_4p5m_shift_m':float(np.percentile([x['matched_progress_4p5m_shift'] for x in rows if 'matched_progress_4p5m_shift' in x],95)),'limitation':'Polyline finite-difference curvature spikes are reference sharpness, not direct physical steering requests. Latest received repeated retained paths may be included.'}
print(summary);(p/'path_audit.json').write_text(json.dumps({'summary':summary,'rows':rows},indent=2))
