import pickle, numpy as np, json, sys
R1=[[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]]; R3=[[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]]
for tag in sys.argv[1:]:
    d=pickle.load(open(f'/home/imac/ros2_ws/drive_debug_20261006_{tag}_light/analysis/decoded.pkl','rb'))
    route=R1 if abs(d['/navigation/global_path'][0]['xy'][0][0]-150)<1 else R3
    od=d['/motive/vehicle/odom_map']; ot=np.array([x['t'] for x in od]); oxy=np.array([x['xy'] for x in od]); oyaw=np.unwrap([x['q'][2] for x in od]); ov=np.array([np.hypot(*x['v']) for x in od])
    gr=d['/bev/occupancy_grid']; gt=np.array([x['t'] for x in gr]); cyc=[]; grids=[]
    for x in d['/debug/upper_branch_event']:
        e=x['data']; t=e['t_start']; i=np.searchsorted(ot,t)-1; j=np.searchsorted(gt,t)-1
        if i<0 or j<0: continue
        cyc.append(dict(t=t,x=float(oxy[i,0]),y=float(oxy[i,1]),yaw=float(np.arctan2(np.sin(oyaw[i]),np.cos(oyaw[i]))),v=float(ov[i]),rec_valid=e['valid_plan'],rec_branch=e['branch_step'],rec_wp0=e['wp0'],rec_planned=e['planned_world'],grid_index=len(grids)))
        grids.append(gr[j]['grid'].astype(np.int8).ravel())
    np.array(grids,np.int8).tofile(f'replay_{tag}_grids.bin'); json.dump({'cycles':cyc,'route':route},open(f'replay_{tag}.json','w')); print(tag,'scenario1' if route is R1 else 'scenario3',len(cyc))
