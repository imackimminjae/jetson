from pathlib import Path
import pickle,json,yaml,numpy as np,hashlib
H=Path(__file__).resolve().parent;R=H.parents[1]
for tag in ['184813','184905','185025','185147']:
 p=R/f'drive_debug_20261006_{tag}_light';d=pickle.load((p/'analysis/decoded.pkl').open('rb'));gs=d['/bev/occupancy_grid'];gt=np.array([g['t'] for g in gs]);bb=d['/debug/upper_interval_centering_state'];bt=np.array([b['data'][0] for b in bb]);tt=d['/debug/upper_miqp_trace'];ts=np.array([v['data'][0] for v in tt]);cs=[];grids=[]
 for z in d['/debug/upper_branch_event']:
  e=z['data'];t=e['t_start']
  if len(e['preview_world'])<3:continue
  j=np.searchsorted(gt,t,side='right')-1;ib=np.argmin(abs(bt-t));it=np.argmin(abs(ts-t));assert j>=0 and abs(bt[ib]-t)<.05
  cs.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(tt[it]['data'][4]),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(bb[ib]['data'][1]),rec_B=float(bb[ib]['data'][2]),grid_index=len(grids),grid_record_index=int(j),grid_age_sec=float(t-gt[j])));grids.append(gs[j]['grid'].astype(np.int8))
 (H/f'input_{tag}.json').write_text(json.dumps({'cycles':cs,'route':d['/navigation/global_path'][0]['xy'].tolist()},indent=2));np.stack(grids).tofile(H/f'grids_{tag}.bin');cfg=yaml.safe_load((p/'config_snapshot.yaml').read_text());cfg['upper_planner_node']['ros__parameters']['grid_value_threshold']=1;(H/f'config_{tag}.yaml').write_text(yaml.safe_dump(cfg));print(tag,len(cs))
