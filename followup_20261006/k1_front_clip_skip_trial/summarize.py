from pathlib import Path
import hashlib,json,numpy as np,yaml
H=Path(__file__).resolve().parent
summary={};total=0
for tag in ['032453','050627','165712','184813','184905','185025','185147']:
 d=json.loads((H/f'result_{tag}.json').read_text())
 old=tag in ['032453','050627','165712']
 p=H.parent/('front_clip_trial' if old else 'origin295_drive_review')
 inp=json.loads((p/f'input_{tag}.json').read_text());cs=inp['cycles']
 grids=np.fromfile(p/f'grids_{tag}.bin',dtype=np.int8).reshape(-1,128,128)
 cfg=yaml.safe_load((p/f'config_{tag}.yaml').read_text())['upper_planner_node']['ros__parameters']
 prev=json.loads((H.parent/'front_clip_trial'/f'result_{tag}.json').read_text())
 first=next((i for i,c in enumerate(cs) if not c['rec_valid']),None)
 row={'cycles':len(cs),'first_recorded_invalid_index':first,'variants':{}}
 for name,rs in d.items():
  base=d[name.split('_')[0]+'_m0'];total+=len(rs)
  changes=[];new_bad=[];old_bad=[];all_blocked=[];viol=[];input_viol=[]
  for i,(c,r,b) in enumerate(zip(cs,rs,base)):
   dp=None
   if r['valid'] and b['valid'] and len(r['planned'])==len(b['planned']):
    dp=float(np.max(np.linalg.norm(np.array(r['planned'])-b['planned'],axis=1)))
   if r['valid']!=b['valid'] or (dp is not None and dp>.001):
    changes.append(dict(i=i,t=c['t'],pose=[c['x'],c['y']],base_valid=b['valid'],valid=r['valid'],max_delta=dp,base_end=b.get('planned',[None])[-1],end=r.get('planned',[None])[-1],skip=r['k1_skip_mask']))
   if not r['valid']:continue
   pos=np.array([c['x'],c['y']]);yaw=c['yaw'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
   body=(np.array(r['planned'])-pos)@R
   N=len(body)-1;dt=cfg.get('upper_prediction_dt_sec',.75);speed=cfg.get('target_speed_mps',6.)
   G=np.zeros((2*N,2*N))
   for k in range(1,N+1):
    for j in range(k):
     for m in range(j,k):
      h=r['psirk'][m]
      G[2*k-2:2*k,2*j:2*j+2]+=dt*np.array([[np.cos(h),-speed*np.sin(h)],[np.sin(h),speed*np.cos(h)]])
   reference=(np.array(r['reference'])-pos)@R
   u=np.linalg.solve(G,(body[1:]-reference[1:]).ravel()).reshape(-1,2)
   h=np.array(r['psirk'][:N]);u[:,1]+=(h-np.r_[0.,h[:-1]]+np.pi)%(2*np.pi)-np.pi
   phi=np.tan(cfg.get('max_steering_angle_rad',.3))*dt/cfg.get('wheelbase',2.8)
   input_viol.append(max(0.,float(np.max(np.abs(u[:,0])-cfg.get('a_max_mps2',1.)*dt)),float(np.max(np.abs(u[:,1])-phi*(speed+np.cumsum(u[:,0]))))))
   def cell_value(world):
    q=(np.array(world)-pos)@R
    x=int(np.floor((q[0]-(0 if old else 2.95))/.3125));y=int(np.floor((q[1]+20)/.3125))
    return int(grids[c['grid_index'],y,x]) if 0<=x<128 and 0<=y<128 else -1
   cell=cell_value(r['planned'][1])
   if cell>=1:
    all_blocked.append(i)
    if b['valid'] and cell_value(b['planned'][1])==0:new_bad.append(i)
   if b['valid'] and cell_value(b['planned'][1])>=1:old_bad.append(i)
   for k,ci in enumerate(r['selected']):
    if k==0 or ci<0:continue
    if k==1 and r['k1_skip_mask'][ci]:continue
    axis=np.array([-np.sin(yaw+r['psirk'][k]),np.cos(yaw+r['psirk'][k])])
    ab=np.array(r['cands'][k][ci]).reshape(2,2)@axis
    val=np.array(r['planned'][k])@axis
    viol.append(max(float(min(ab)-val),float(val-max(ab)),0))
  row['variants'][name]=dict(valid=sum(r['valid'] for r in rs),first_failure_valid=rs[first]['valid'] if first is not None else None,skip_cycles=sum(any(r['k1_skip_mask']) for r in rs),changes=changes,k1_blocked=all_blocked,k1_new_blocked_vs_valid_free_base=new_bad,max_retained_lane_violation=max(viol,default=0),max_input_violation=max(input_viol,default=0))
 row['baseline_repro']={}
 for name in ['fixed_m0','carried_m0']:
  dif=[]
  for a,b in zip(d[name],prev[name]):
   assert a['valid']==b['valid'],(tag,name)
   if a['valid']:
    assert len(a['planned'])==len(b['planned'])
    dif.append(float(np.max(np.abs(np.array(a['planned'])-b['planned']))))
  row['baseline_repro'][name]=max(dif,default=0)
 summary[tag]=row
integrity={k:hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in json.loads((H/'sources_before.json').read_text()).items()}
(H/'summary.json').write_text(json.dumps(dict(total_solves=total,production_unchanged=integrity,results=summary),indent=2))
print('solves',total,'production unchanged',all(integrity.values()))
for tag,row in summary.items():
 print(tag,'valid',{k:v['valid'] for k,v in row['variants'].items()},'first failure',{k:v['first_failure_valid'] for k,v in row['variants'].items()},'skip',{k:v['skip_cycles'] for k,v in row['variants'].items()})
