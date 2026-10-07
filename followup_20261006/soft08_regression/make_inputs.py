import json
V='/home/imac/ros2_ws/followup_20261006/verify/'
CFG='/home/imac/ros2_ws/followup_20261006/soft08_regression/config_current.yaml'
V0=dict(name='V0_validated',soft_ratio=0.7,upper_r_dpsi=3.0,wp_switch_eps=10.0,wp_switch_max_distance_m=15.0,lower_rd_steering_rate=120.0,plan_period_steps=10)
CUR=dict(name='CUR',plan_period_steps=5)  # YAML: soft .8, r_dpsi 2, wp 15/20, rd 150
configs=[V0,CUR,
 dict(CUR,name='CUR_soft07',soft_ratio=0.7),
 dict(CUR,name='CUR_r3',upper_r_dpsi=3.0),
 dict(CUR,name='CUR_1hz',plan_period_steps=10),
 dict(CUR,name='CUR_wp1015',wp_switch_eps=10.0,wp_switch_max_distance_m=15.0)]
for suite in ['synthetic','ysynthetic','replica','recorded','robust']:
    d=json.load(open(V+f'input_{suite}_prod.json'))
    d['config_file']=CFG; d['configs']=configs; d['live_extra_configs']=[]
    json.dump(d,open(f'input_{suite}.json','w'))
d=json.load(open(V+'input_synthetic_prod.json')); d['config_file']=CFG; d['configs']=[V0]; json.dump(d,open('input_repro.json','w'))
