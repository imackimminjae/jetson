from pathlib import Path
import json,pickle,numpy as np,yaml,shutil,hashlib
H=Path(__file__).resolve().parent;R=H.parents[1]
tags=['034205','035257','050536','050627','055105','165712']
manifest=[]
current=yaml.safe_load((R/'src/virtual_control/config/tracking_control_split.yaml').read_text())
for tag in tags:
 if tag in ['034205','035257']:
  p=R/f'drive_debug_20261006_{tag}_light';d=pickle.load((p/'analysis/decoded.pkl').open('rb'))
  gg=d['/bev/occupancy_grid'];gt=np.array([g['t'] for g in gg]);bb=d['/debug/upper_interval_centering_state'];bt=np.array([b['data'][0] for b in bb]);tt=d['/debug/upper_miqp_trace'];ts=np.array([z['data'][0] for z in tt]);cs=[];grids=[]
  for z in d['/debug/upper_branch_event']:
   e=z['data'];t=e['t_start']
   if len(e['preview_world'])<3:continue
   j=np.searchsorted(gt,t,side='right')-1;ib=np.argmin(abs(bt-t));it=np.argmin(abs(ts-t));assert j>=0 and abs(bt[ib]-t)<.05
   cs.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(tt[it]['data'][4]),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(bb[ib]['data'][1]),rec_B=float(bb[ib]['data'][2]),grid_index=len(grids),grid_record_index=int(j)))
   grids.append(gg[j]['grid'].astype(np.int8))
  (H/f'input_{tag}.json').write_text(json.dumps(dict(cycles=cs,route=d['/navigation/global_path'][0]['xy'].tolist())))
  np.stack(grids).tofile(H/f'grids_{tag}.bin')
  cfg=yaml.safe_load((p/'config_snapshot.yaml').read_text())
 else:
  if tag=='165712':
   p=H.parent/'front_clip_trial';inp=p/f'input_{tag}.json';grid=p/f'grids_{tag}.bin'
  else:
   p=H.parent/'reference_B_separation';inp=p/f'replay_{tag}.json';grid=p/f'replay_{tag}_grids.bin'
  shutil.copy2(inp,H/f'input_{tag}.json');shutil.copy2(grid,H/f'grids_{tag}.bin')
  cfg=yaml.safe_load((p/f'config_{tag}.yaml').read_text())
 for kind,c in [('recorded',cfg),('current',current)]:
  c=json.loads(json.dumps(c));pars=c['upper_planner_node']['ros__parameters'];pars['grid_value_threshold']=1
  (H/f'config_{tag}_{kind}.yaml').write_text(yaml.safe_dump(c))
  raw_origin=2.95 if tag=='165712' else 0.0
  origin=pars.get('grid_origin_x_override_m',0.) if pars.get('grid_origin_x_override_enabled',False) else raw_origin
  specs=[dict(name=('fixed' if legacy else 'carried')+f'_m{mode}',legacy=legacy,fixed_wp=True,grid_origin_x=origin,skip_k1_mode=mode) for legacy in [1,0] for mode in [0,1,2]]
  (H/f'specs_{tag}_{kind}.json').write_text(json.dumps(specs,indent=2))
  manifest.append(dict(tag=tag,config=kind,raw_origin=raw_origin,effective_origin=origin,cycles=len(json.loads((H/f'input_{tag}.json').read_text())['cycles'])))
(H/'manifest.json').write_text(json.dumps(manifest,indent=2))
files=[R/'src/virtual_control/config/tracking_control_split.yaml',H.parent/'k1_front_clip_skip_trial/replay']
(H/'sources.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
print(manifest)
