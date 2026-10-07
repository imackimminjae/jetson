from pathlib import Path
import shutil,json
h=Path(__file__).resolve().parent;old=h.parent/'reference_B_separation';base=h.parent/'near_consistency'
s=(base/'sim.cpp').read_text().replace('extern double experiment_near_weight;','extern double experiment_soft_max_width_m;\nextern double experiment_force_B;')
s=s.replace('experiment_near_weight=c.value("near_weight",0.0);','experiment_soft_max_width_m=0.0;experiment_force_B=-1.0;n.upper_preview_steps_=c.value("upper_preview_steps",n.upper_preview_steps_);')
s=s.replace(' static J plan(Node&n,',' static int plannerTicks(Node&n){return std::max(1,static_cast<int>(std::lround(10.0/n.upper_planner_rate_hz_)));}\n static J plan(Node&n,')
assert s.count('step%10==0')==1
s=s.replace('step%10==0','step%UA::plannerTicks(*upper)==0')
(h/'sim.cpp').write_text(s)
s=(old/'build.py').read_text();s=s.replace("['tracking_control_trial','replay']","['sim']").replace("str(here/'tracking_control_trial.o')","str(here.parent/'reference_B_separation/tracking_control_trial.o')")
(h/'build_sim.py').write_text(s)
for group in ['synthetic','replica','ysynthetic','robust']:
 d=json.loads((h.parent/'verify'/f'input_{group}_prod.json').read_text());d['configs']=[dict(name='h7',upper_preview_steps=7),dict(name='h6',upper_preview_steps=6)];d['live_extra_configs']=[];d['config_file']=str(h/'config_055105_current_h7.yaml');(h/f'input_{group}.json').write_text(json.dumps(d))
print('Prepared 2Hz upper / 10Hz lower isolated numerical simulator. Output backends disabled by existing harness.')
