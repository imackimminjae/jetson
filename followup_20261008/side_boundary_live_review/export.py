from pathlib import Path
import json,pickle,numpy as np,shutil,hashlib
H=Path(__file__).resolve().parent;R=H.parents[1];tags=['20261008_052329','20261008_052412'];hashes={};manifest=[]
for tag in tags:
 P=R/f'drive_debug_{tag}_light';f=P/'analysis/decoded.pkl'
 if not f.exists():continue
 d=pickle.load(f.open('rb'));gr=d['/bev/occupancy_grid'];gt=np.array([z['t'] for z in gr]);od=d['/motive/vehicle/odom_map'];ot=np.array([z['t'] for z in od]);cs=np.array([z['data'] for z in d['/debug/upper_interval_centering_state']]);cycles=[];grids=[];prev=[]
 for i,z in enumerate(d['/debug/upper_branch_event']):
  e=z['data'];t=e['t_start'];gi=np.searchsorted(gt,t)-1;oi=np.searchsorted(ot,t)-1;ci=np.argmin(abs(cs[:,0]-t))
  if gi<0 or oi<0 or not e['preview_world']:continue
  g=gr[gi];assert g['width']==128 and g['height']==128 and g['origin']==[0,-20],(tag,g['origin']);assert abs(cs[ci,0]-t)<.1
  cycles.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(np.hypot(*od[oi]['v'])),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(cs[ci,1]),rec_B=float(cs[ci,2]),rec_used_previous=len(prev)==len(e['preview_world']),grid_index=len(grids),event_index=i,grid_record_index=int(gi)))
  grids.append(g['grid'].astype(np.int8).ravel())
  if e['valid_plan']:prev=e['planned_world']
 route=d['/navigation/global_path'][0]['xy'];np.array(grids,np.int8).tofile(H/f'grids_{tag}.bin');(H/f'input_{tag}.json').write_text(json.dumps(dict(cycles=cycles,route=route.tolist())));shutil.copy2(P/'config_snapshot.yaml',H/f'config_{tag}.yaml');manifest.append(tag)
 for file in [P/'bag/bag_0.db3',P/'config_snapshot.yaml']:hashes[str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
 print(tag,len(cycles))
(H/'tags.json').write_text(json.dumps(manifest));(H/'inputs_sha256.json').write_text(json.dumps(hashes,indent=2));(H/'specs.json').write_text(json.dumps([dict(name='fixed_off',legacy=1,guard=False),dict(name='fixed_on',legacy=1,guard=True),dict(name='carried_off',guard=False),dict(name='carried_on',guard=True)]))
