from pathlib import Path
import json,pickle,hashlib,numpy as np,yaml
H=Path(__file__).resolve().parent;R=H.parents[1];old=H.parent/'reference_B_separation';manifest={}
for n,v in json.loads((old/'baseline/sha256.json').read_text()).items():
 p=R/n;cur=hashlib.sha256(p.read_bytes()).hexdigest();manifest[str(p)]=cur
 if 'tracking_control' in n and p.suffix in ['.cpp','.hpp']:assert cur==v,(n,'source changed')
for tag in ['175433','174526']:
 bag=R/f'drive_debug_20261006_{tag}_light';d=pickle.load((bag/'analysis/decoded.pkl').open('rb'));gr=d['/bev/occupancy_grid'];gt=np.array([v['t'] for v in gr]);od=d['/motive/vehicle/odom_map'];ot=np.array([v['t'] for v in od]);cs=d['/debug/upper_interval_centering_state'];ct=np.array([v['data'][0] for v in cs]);ut=d['/debug/upper_miqp_trace'];tt=np.array([v['data'][0] for v in ut]);cycles=[];grids=[]
 for ev in d['/debug/upper_branch_event']:
  e=ev['data'];t=e['t_start'];j=np.searchsorted(gt,t,side='right')-1;i=np.searchsorted(ot,t,side='right')-1
  if i<0 or j<0 or not e['preview_world']:continue
  g=gr[j];assert g['origin']==[2.95,-20] and g['res']==.3125 and g['grid'].shape==(128,128)
  ci=int(np.argmin(abs(ct-t)));ui=int(np.argmin(abs(tt-t)));assert abs(ct[ci]-t)<.05 and abs(tt[ui]-t)<.05
  cycles.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(ut[ui]['data'][4]),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(cs[ci]['data'][1]),rec_B=float(cs[ci]['data'][2]),grid_index=len(grids),grid_record_index=int(j),grid_age_sec=float(t-gt[j])))
  grids.append(g['grid'].astype(np.int8))
 orig=np.stack(grids);crop=orig.copy();crop[:,:,106:]=-1
 assert np.array_equal(orig[:,:,:106],crop[:,:,:106]);assert np.all(crop[:,:,106:]==-1)
 for mode,arr in [('full40',orig),('mask33125',crop)]:arr.tofile(H/f'{tag}_{mode}.bin')
 (H/f'{tag}_input.json').write_text(json.dumps({'cycles':cycles,'route':d['/navigation/global_path'][0]['xy'].tolist()},indent=2))
 cfg=yaml.safe_load((bag/'config_snapshot.yaml').read_text());cfg['upper_planner_node']['ros__parameters']['grid_value_threshold']=1
 (H/f'{tag}_config.yaml').write_text(yaml.safe_dump(cfg))
 for p in [bag/'bag/bag_0.db3',bag/'config_snapshot.yaml',bag/'analysis/decoded.pkl']:manifest[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
 print(tag,'cycles',len(cycles),'original entries changed',int(np.sum(orig!=crop)),'max grid age',max(c['grid_age_sec'] for c in cycles))
for p in [old/'replay',old/'replay.cpp',old/'tracking_control_trial.cpp']:manifest[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(H/'sources_before_sha256.json').write_text(json.dumps(manifest,indent=2));(H/'specs.json').write_text(json.dumps([{'name':'fixed','legacy':1},{'name':'carried','fixed_wp':True}],indent=2))
(H/'EXPERIMENT.txt').write_text('Only grid columns106..127 changed to -1. 128x128 resolution0.3125 and effective planner origin(0,-20), Nmax7 unchanged. Cutoff is x=33.125m in effective planner coordinates, NOT received origin2.95 coordinates. No lateral cropping or rescaling. Old projection is not reconstructed. Fixed: recorded reference/B-before/wp each cycle; carried: own plan/B after recorded initial seed, recorded wp/vehicle pose/grid each cycle. Diagnostic cap=0 and forceB=-1. Replay node uses domain177 localhost1; no spin, timer cancelled, no hardware interfaces.\n')
