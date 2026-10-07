import pickle, numpy as np, json, sys
for tag,p in (('R3','/home/imac/ros2_ws/drive_debug_20261005_234518_light/analysis/decoded.pkl'),('A','/home/imac/ros2_ws/drive_debug_20261003_055920_light/analysis/decoded.pkl')):
    d=pickle.load(open(p,'rb')); od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); oyaw=np.unwrap([x['q'][2] for x in od]); ov=np.array([np.hypot(*x['v']) for x in od])
    gr=d['/bev/occupancy_grid']; gt=np.array([x['t'] for x in gr]); cycles=[]; grids=[]
    for x in d['/debug/upper_branch_event']:
        e=x['data']; t=e['t_start']
        i=np.searchsorted(ot,t)-1; j=np.searchsorted(gt,t)-1
        if i<0 or j<0: continue
        g=gr[j]; assert g['width']==128 and abs(g['res']-0.3125)<1e-9 and g['origin']==[0.0,-20.0]
        cycles.append(dict(t=t,x=float(oxy[i,0]),y=float(oxy[i,1]),yaw=float(np.arctan2(np.sin(oyaw[i]),np.cos(oyaw[i]))),v=float(ov[i]),
                           rec_valid=e['valid_plan'],rec_branch=e['branch_step'],rec_wp0=e['wp0'],rec_planned=e['planned_world'],grid_index=len(grids)))
        grids.append(g['grid'].astype(np.int8).ravel())
    np.array(grids,np.int8).tofile(f'replay_{tag}_grids.bin'); json.dump({'cycles':cycles,'route':[[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]]},open(f'replay_{tag}.json','w'))
    print(tag,len(cycles))
