import json, pickle, numpy as np
meta=json.load(open('recorded_world_meta.json'))
GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
# dense center along route (only for harness bookkeeping; widths huge so no left-road stop)
center=[];arc=[];s=0.0
for a,b in zip(GP[:-1],GP[1:]):
    n=int(np.ceil(np.linalg.norm(b-a)/0.5))
    for i in range(n):
        q=a+(b-a)*i/n
        if center: s+=float(np.linalg.norm(q-np.array(center[-1])))
        center.append(q.tolist()); arc.append(s)
SER=pickle.load(open('../runs_series.pkl','rb')); R=SER['R3_1005_2345_sc1']
scenes=[]
for prog in (45.0,60.0):
    i=int(np.argmin(np.abs(R['prog']-prog))); j=min(i+5,len(R['mx'])-1)
    x,y=R['mx'][i],R['my'][i]; yaw=float(np.arctan2(R['my'][j]-y,R['mx'][j]-x))
    for dyaw in (-3.0,0.0,3.0):
        scenes.append({'name':f'recorded_p{int(prog)}_yaw{dyaw:+.0f}','route':GP.tolist(),'center':center,'arc':arc,'widths':[200.0]*len(center),'spur':[],
            'bitmap':'/home/imac/ros2_ws/followup_20261006/verify/recorded_world.bin','origin':meta['origin'],'resolution':meta['resolution'],'width':meta['width'],'height':meta['height'],
            'start':[float(x),float(y),yaw+np.radians(dyaw)],'v0':5.8,'duration':26,'mode':'live'})
old=json.load(open('/home/imac/ros2_ws/rollback/20261003_steering_response_model/validation_input.json'))
synthetic=[s for s in old['scenes'] if s['name'] in ('live_curve','branch_width')]
configs=[{'name':'production','branch_variant':0},{'name':'V1_truncate','branch_variant':1},{'name':'V2_truncate_split_turn','branch_variant':2}]
base={'plants':[{'name':'measured_slow','tau':0.74,'delay':0,'yaw_noise_deg':0}],'configs':configs,'live_extra_configs':[],
      'config_file':'/home/imac/ros2_ws/install/virtual_control/share/virtual_control/config/tracking_control_split.yaml'}
json.dump({**base,'scenes':scenes},open('input_recorded.json','w'))
json.dump({**base,'scenes':synthetic},open('input_synthetic.json','w'))
print([s['name'] for s in scenes],[ (round(s['start'][0],1),round(s['start'][1],1),round(np.degrees(s['start'][2]),1)) for s in scenes])
