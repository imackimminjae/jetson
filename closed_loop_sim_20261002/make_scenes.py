from pathlib import Path
import json, pickle
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree

p=Path(__file__).resolve().parent
bag=pickle.load(open(p.parent/'drive_debug_20261001_184521_light/analysis/decoded.pkl','rb'))
original=np.array(bag['/navigation/global_path'][0]['xy'])
route=np.column_stack([original[0,1]-original[:,1],original[:,0]-original[0,0]])
scenes=[]
for name in ['straight','curve','branch_width']:
    wp=np.column_stack([np.arange(0,181,30),np.zeros(7)]) if name=='straight' else route*1.4
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(wp,axis=0),axis=1))]
    parameter=np.arange(0,distance[-1],.2);parameter=np.r_[parameter,distance[-1]]
    spline=CubicSpline(distance,wp,axis=0,bc_type='natural')
    center=spline(parameter); arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(center,axis=0),axis=1))]
    widths=np.full(len(center),10.)
    if name=='branch_width': widths=10-3*np.exp(-((arc-65)/14)**2)+3*np.exp(-((arc-110)/16)**2)
    direction=(center[-1]-center[-2]);direction/=np.linalg.norm(direction)
    first=(center[1]-center[0]);first/=np.linalg.norm(first)
    road=np.vstack([center[0]-np.arange(20,.1,-.2)[:,None]*first,center,center[-1]+np.arange(.2,30,.2)[:,None]*direction])
    rw=np.r_[np.full(100,widths[0]),widths,np.full(149,widths[-1])]
    assert len(road)==len(rw)
    spur=[]
    if name=='branch_width':
        vec=wp[5]-wp[4];vec/=np.linalg.norm(vec)
        spur=(wp[5]+np.arange(0,46,.2)[:,None]*vec).tolist()
        road=np.vstack([road,spur]);rw=np.r_[rw,np.full(len(spur),8.)]
    origin=np.floor(road.min(axis=0)-30);upper=np.ceil(road.max(axis=0)+30)
    res=.1;size=np.ceil((upper-origin)/res).astype(int)
    tree=cKDTree(road);image=np.full((size[1],size[0]),100,dtype=np.int8)
    for start in range(0,size[1],100):
        X,Y=np.meshgrid(origin[0]+(np.arange(size[0])+.5)*res,origin[1]+(np.arange(start,min(start+100,size[1]))+.5)*res)
        pts=np.column_stack([X.ravel(),Y.ravel()]);dist,idx=tree.query(pts)
        image[start:start+len(Y)]=np.where(dist.reshape(X.shape)<=rw[idx].reshape(X.shape)/2,0,100)
    filename=p/(name+'.bin');image.tofile(filename)
    scenes.append(dict(name=name,route=wp.tolist(),center=center.tolist(),arc=arc.tolist(),widths=widths.tolist(),spur=spur,
                       bitmap=str(filename),origin=origin.tolist(),resolution=res,width=int(size[0]),height=int(size[1]),
                       start=[float(center[0,0]),float(center[0,1]),float(np.arctan2(first[1],first[0]))],duration=48))
configs=[
    {'name':'baseline'}, {'name':'turn03','lambda_relative_turn':.3}, {'name':'turn015','lambda_relative_turn':.15},
    {'name':'turn0','lambda_relative_turn':0.}, {'name':'dpsi2','upper_r_dpsi':2.},
    {'name':'terminal100','terminal_position_weight_multiplier':100.}, {'name':'wp6','wp_switch_eps':6.},
    {'name':'samples4','preview_min_segment_samples':4}, {'name':'lower80','lower_rd_steering_rate':80.},
    {'name':'filter09','enable_output_filter':True,'steer_norm_rate_max':3.,'steer_norm_lpf_alpha':.9}]
plants=[{'name':'nominal','tau':.05,'delay':0.,'yaw_noise_deg':0.},
        {'name':'perturbed','tau':.08,'delay':.02,'yaw_noise_deg':1.}]
d={'scenes':scenes,'configs':configs,'plants':plants,'config_file':str(p.parent/'drive_debug_20261001_184521_light/config_snapshot.yaml')}
(p/'experiments.json').write_text(json.dumps(d))
smoke={**d,'configs':[configs[0]],'plants':[plants[0]]};(p/'smoke.json').write_text(json.dumps(smoke))
print('Created three static world roads and 60 paired experiments')
