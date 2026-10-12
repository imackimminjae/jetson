from pathlib import Path
import json,pickle,numpy as np,yaml,hashlib
H=Path(__file__).resolve().parent;R=H.parents[1]
s=(H.parent/'bev_registration/replay.cpp').read_text();needle='   n.applyTerminalPositionTargets(pc,';assert s.count(needle)==1
patch='''   // Offline-only diagnostic: remove inset at the front map clipping edge.
   J adjusted=J::array();
   if(spec.value("front_clip_mode",0)>0){
    const int mode=spec["front_clip_mode"];
    for(size_t k=1;k<pc.pmk.size();++k)for(size_t i=0;i<pc.pmk[k].size();++i){
     if(mode==3 && k!=1)continue;
     const auto& info=pc.interval_info[k][i];
     if(info.observation.nominal_fallback)continue;
     V a=pc.pmk[k][i],b=pc.pMk[k][i],u=(b-a).normalized();
     const double inset=info.treatment.inset;
     V raw_a=a-inset*u,raw_b=b+inset*u;
     const bool front_a=!info.observation.start_boundary_observed && std::abs(raw_a.x()-g.origin_body.x())<0.11;
     const bool front_b=!info.observation.end_boundary_observed && std::abs(raw_b.x()-g.origin_body.x())<0.11;
     if(front_a||front_b){
      if(front_a||mode==2)pc.pmk[k][i]=raw_a;
      if(front_b||mode==2)pc.pMk[k][i]=raw_b;
      adjusted.push_back({{"k",k},{"i",i},{"front_a",front_a},{"front_b",front_b},{"inset",inset}});
     }
    }
   }
'''
s=s.replace(needle,patch+needle).replace('   row["position_targets"]=', '   row["front_clip_adjustments"]=adjusted;\n   row["position_targets"]=');(H/'replay.cpp').write_text(s);(H/'build_replay.py').write_text((H.parent/'bev_registration/build_replay.py').read_text())
for tag in ['032453','050627','165712']:
 p=R/f'drive_debug_20261006_{tag}_light';d=pickle.load((p/'analysis/decoded.pkl').open('rb'));gg=d['/bev/occupancy_grid'];gt=np.array([g['t'] for g in gg]);bb=d['/debug/upper_interval_centering_state'];bt=np.array([b['data'][0] for b in bb]);tt=d['/debug/upper_miqp_trace'];ts=np.array([z['data'][0] for z in tt]);cs=[];grids=[]
 for z in d['/debug/upper_branch_event']:
  e=z['data'];t=e['t_start']
  if len(e['preview_world'])<3:continue
  j=np.searchsorted(gt,t,side='right')-1;ib=np.argmin(abs(bt-t));it=np.argmin(abs(ts-t));assert j>=0 and abs(bt[ib]-t)<.05
  cs.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(tt[it]['data'][4]),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(bb[ib]['data'][1]),rec_B=float(bb[ib]['data'][2]),grid_index=len(grids),grid_record_index=int(j)));grids.append(gg[j]['grid'].astype(np.int8))
 (H/f'input_{tag}.json').write_text(json.dumps({'cycles':cs,'route':d['/navigation/global_path'][0]['xy'].tolist()},indent=2));np.stack(grids).tofile(H/f'grids_{tag}.bin');cfg=yaml.safe_load((p/'config_snapshot.yaml').read_text());cfg['upper_planner_node']['ros__parameters']['grid_value_threshold']=1;(H/f'config_{tag}.yaml').write_text(yaml.safe_dump(cfg))
for origin in [0,2.95]:
 specs=[]
 for legacy in [1,0]:
  for mode in [0,1,2,3]:specs.append({'name':('fixed' if legacy else 'carried')+f'_m{mode}','legacy':legacy,'fixed_wp':True,'grid_origin_x':origin,'front_clip_mode':mode})
 (H/f'specs_{origin}.json').write_text(json.dumps(specs,indent=2))
files=[R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/config/tracking_control_split.yaml'];(H/'sources_before.json').write_text(json.dumps({str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files},indent=2))
