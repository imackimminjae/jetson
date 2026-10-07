from pathlib import Path
import json,pickle,numpy as np,yaml,hashlib,difflib,shutil
H=Path(__file__).resolve().parent;R=H.parents[1];O=H.parent/'reference_B_separation';bag=R/'drive_debug_20261006_180803_light';d=pickle.load((bag/'analysis/decoded.pkl').open('rb'));g=d['/bev/occupancy_grid'];gt=np.array([v['t'] for v in g]);B=d['/debug/upper_interval_centering_state'];bt=np.array([v['data'][0] for v in B]);T=d['/debug/upper_miqp_trace'];tt=np.array([v['data'][0] for v in T]);cs=[];arr=[]
for v in d['/debug/upper_branch_event']:
 e=v['data'];t=e['t_start'];j=np.searchsorted(gt,t,side='right')-1;ib=np.argmin(abs(bt-t));it=np.argmin(abs(tt-t));assert j>=0 and abs(bt[ib]-t)<.05
 cs.append(dict(t=t,x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(T[it]['data'][4]),rec_wp0=e['wp0'],rec_wp1=e['wp1'],rec_valid=e['valid_plan'],rec_planned=e['planned_world'],rec_preview=e['preview_world'],rec_B_before=float(B[ib]['data'][1]),rec_B=float(B[ib]['data'][2]),grid_index=len(arr),grid_record_index=int(j),grid_age_sec=float(t-gt[j])))
 arr.append(g[j]['grid'].astype(np.int8))
(H/'input.json').write_text(json.dumps({'cycles':cs,'route':d['/navigation/global_path'][0]['xy'].tolist()},indent=2));np.stack(arr).tofile(H/'grids.bin');cfg=yaml.safe_load((bag/'config_snapshot.yaml').read_text());cfg['upper_planner_node']['ros__parameters']['grid_value_threshold']=1;(H/'config.yaml').write_text(yaml.safe_dump(cfg))
# Isolated diagnostic source; all experiment knobs disabled for baseline.
s=(O/'tracking_control_trial.cpp').read_text();s=s.replace('double experiment_force_B =','int experiment_force_turn_step = -1;\ndouble experiment_force_B =');needle='  if (result.branch_step >= 1 && lambda_relative_turn_ > 0.0) {';assert s.count(needle)==1;s=s.replace(needle,'  if (experiment_force_turn_step > 0) result.branch_step = std::min(horizon, experiment_force_turn_step);\n'+needle);(H/'tracking_control_trial.cpp').write_text(s)
s=(O/'replay.cpp').read_text().replace('extern double experiment_force_B;','extern double experiment_force_B;\nextern int experiment_force_turn_step;');s=s.replace('  Points route;', '  experiment_force_turn_step=spec.value("force_turn_step",-1);\n  if(spec.contains("lambda_turn"))n.lambda_relative_turn_=spec["lambda_turn"];\n  Points route;')
s=s.replace('   experiment_force_B=fixed_B?c["rec_B"].get<double>():-1.0;', '   experiment_force_B=spec.value("force_B",fixed_B?c["rec_B"].get<double>():-1.0);\n   if(spec.value("hold_wp5",false) && wp>=6 && position.x()<280) {wp=5;pair[0]=5;pair[1]=6;}')
s=s.replace('   auto ref=n.buildPreviewReference(', '   if(spec.value("fresh_ref",false)){prefix.clear();used=false;previous.clear();}\n   auto ref=n.buildPreviewReference(')
s=s.replace('   n.applyTerminalPositionTargets(pc,', '''   if(spec.value("raw_corridors",false)){
    for(size_t k=0;k<pc.pmk.size();++k)for(size_t i=0;i<pc.pmk[k].size();++i){
     V a=pc.pmk[k][i],b=pc.pMk[k][i],u=(b-a).normalized(),mid=.5*(a+b);double raw=pc.raw_lengths[k][i];
     pc.pmk[k][i]=mid-.5*raw*u;pc.pMk[k][i]=mid+.5*raw*u;
    }
   }
   n.applyTerminalPositionTargets(pc,''')
s=s.replace('   row["used_previous"]=used;', '''   row["position_targets"]=J::array();for(const auto&q:pc.position_targets){V p=position+R*q;row["position_targets"].push_back({p.x(),p.y()});}
   row["input_cost"]=z.input_cost;row["objective"]=z.objective;row["psirk"]=pc.psirk;
   row["used_previous"]=used;''')
(H/'replay.cpp').write_text(s);shutil.copy2(O/'build.py',H/'build.py')
sp=[{'name':'baseline','legacy':1},{'name':'r2','legacy':1,'r_dpsi':2},{'name':'B5','legacy':1,'force_B':5.0},{'name':'raw_corridors','legacy':1,'raw_corridors':True},{'name':'fresh_ref','legacy':1,'fresh_ref':True},{'name':'force_turn4','legacy':1,'force_turn_step':4},{'name':'turn_weight10','legacy':1,'lambda_turn':6.0},{'name':'hold_wp5','legacy':1,'hold_wp5':True},{'name':'hold_wp5_turn4','legacy':1,'hold_wp5':True,'force_turn_step':4}];(H/'specs.json').write_text(json.dumps(sp,indent=2))
files=[bag/'bag/bag_0.db3',bag/'config_snapshot.yaml',R/'src/virtual_control/src/tracking_control.cpp',R/'src/virtual_control/include/virtual_control/tracking_control.hpp',R/'src/virtual_control/include/virtual_control/egocentric_planner_geometry.hpp'];(H/'sources_sha256.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2));(H/'diagnostic.patch').write_text(''.join(difflib.unified_diff((R/'src/virtual_control/src/tracking_control.cpp').read_text().splitlines(True),(H/'tracking_control_trial.cpp').read_text().splitlines(True),fromfile='production',tofile='diagnostic')));print('prepared',len(cs),'cycles',len(sp),'variants')
