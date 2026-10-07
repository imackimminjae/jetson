import json,copy
d=json.load(open('input_synthetic.json')); base=d['scenes'][1]; assert base['name']=='branch_width'
scenes=[]
for off in (-0.6,0.0,0.6):
    for dyaw in (-2.0,0.0,2.0):
        s=copy.deepcopy(base); s['name']=f'branch_width_off{off:+.1f}_yaw{dyaw:+.0f}'
        # lateral offset: route starts heading ~east, so offset in y
        s['start']=[base['start'][0],base['start'][1]+off,base['start'][2]+dyaw*3.14159265/180]; scenes.append(s)
d['scenes']=scenes
d['plants']=[dict(name='tau0.74',tau=0.74,delay=0,yaw_noise_deg=0),dict(name='tau0.5',tau=0.5,delay=0,yaw_noise_deg=0),dict(name='tau1.0',tau=1.0,delay=0,yaw_noise_deg=0)]
V0=dict(name='V0',soft_ratio=0.7,upper_r_dpsi=3.0,wp_switch_eps=10.0,wp_switch_max_distance_m=15.0,lower_rd_steering_rate=120.0,plan_period_steps=10)
d['configs']=[V0,dict(V0,name='V0+2hz',plan_period_steps=5),dict(V0,name='V0+soft08',soft_ratio=0.8),dict(V0,name='V0+r2',upper_r_dpsi=2.0),
 dict(V0,name='V0+wp1520',wp_switch_eps=15.0,wp_switch_max_distance_m=20.0),dict(V0,name='V0+rd150',lower_rd_steering_rate=150.0),
 dict(name='CUR',plan_period_steps=5),dict(name='CUR_1hz_r3_soft07',plan_period_steps=10,upper_r_dpsi=3.0,soft_ratio=0.7)]
json.dump(d,open('input_sharp.json','w'))
print(len(scenes)*3*len(d['configs']),'runs')
