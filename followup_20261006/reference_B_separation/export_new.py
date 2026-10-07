from pathlib import Path
import pickle,numpy as np,json,hashlib,shutil
root=Path('/home/imac/ros2_ws');here=Path(__file__).resolve().parent
manifest={}
for tag in ['055105']:
 bag=root/f'drive_debug_20261006_{tag}_light';p=bag/'analysis/decoded.pkl'
 manifest[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();d=pickle.load(p.open('rb'))
 gr=d['/bev/occupancy_grid'];gt=np.array([r['t'] for r in gr]); od=d['/motive/vehicle/odom_map'];ot=np.array([r['t'] for r in od]);cs=d['/debug/upper_interval_centering_state'];ct=np.array([r['data'][0] for r in cs]);cycles=[];grids=[]
 for ev in d['/debug/upper_branch_event']:
  e=ev['data'];t=e['t_start'];i=np.searchsorted(ot,t)-1;j=np.searchsorted(gt,t)-1
  if i<0 or j<0 or not e['preview_world']:continue
  g=gr[j];assert g['width']==g['height']==128 and abs(g['res']-.3125)<1e-9 and g['origin']==[0,-20]
  ci=int(np.argmin(abs(ct-t)));assert abs(ct[ci]-t)<.05
  cycles.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(np.hypot(*od[i]['v'])),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(cs[ci]['data'][1]),rec_B=float(cs[ci]['data'][2]),grid_index=len(grids)))
  grids.append(g['grid'].astype(np.int8).ravel())
 route=d['/navigation/global_path'][0]['xy']
 np.array(grids,np.int8).tofile(here/f'replay_{tag}_grids.bin')
 (here/f'replay_{tag}.json').write_text(json.dumps(dict(cycles=cycles,route=np.array(route).tolist())))
 shutil.copy2(bag/'config_snapshot.yaml',here/f'config_{tag}.yaml')
 manifest[str(bag/'config_snapshot.yaml')]=hashlib.sha256((bag/'config_snapshot.yaml').read_bytes()).hexdigest()
 print(tag,len(cycles),flush=True)
(here/'input_source_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
shutil.copy2(here/'baseline/src/virtual_control/config/tracking_control_split.yaml',here/'config_current.yaml')
