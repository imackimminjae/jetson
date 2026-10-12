from pathlib import Path
import json
H=Path(__file__).resolve().parent
s=(H.parent/'bev_registration/replay.cpp').read_text()
# Compatibility variables used by this shared harness; no hooks in production.
s=s.replace('extern double experiment_soft_max_width_m;', 'double experiment_soft_max_width_m=0.0;')
s=s.replace('extern double experiment_force_B;', 'double experiment_force_B=-1.0;')
s=s.replace('extern int experiment_force_turn_step;', 'int experiment_force_turn_step=-1;')
s=s.replace('   row["position_targets"]=', '   row["k1_skip_mask"]=pc.k1_lane_constraint_excluded;\n   row["position_targets"]=')
needle='  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);'
assert s.count(needle)==1
s=s.replace(needle,'''  opt.parameter_overrides({rclcpp::Parameter("skip_front_clipped_k1_lane_constraints",spec.value("skip_k1_mode",0)==1)});
'''+needle)
(H/'production_replay.cpp').write_text(s)
for origin in [0,2.95]:
 specs=json.loads((H/f'specs_{origin}.json').read_text())
 (H/f'production_specs_{origin}.json').write_text(json.dumps([s for s in specs if s['skip_k1_mode']!=2],indent=2))
s=(H/'run.py').read_text().replace("H/'replay'", "H/'production_replay'").replace("f'result_", "f'production_result_").replace("f'specs_", "f'production_specs_")
(H/'run_production_replay.py').write_text(s)
