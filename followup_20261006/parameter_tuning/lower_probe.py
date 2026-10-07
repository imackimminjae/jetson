"""Replay identical recorded plans/speed into free simulated vehicles; no road-safety claim."""
from pathlib import Path
import json,pickle,os,subprocess,sys
import numpy as np
here=Path(__file__).resolve().parent;root=here.parents[1]
configs=[{'name':'baseline'}, {'name':'lower_rd180','lower_rd_steering_rate':180.},
    {'name':'lower_rd240','lower_rd_steering_rate':240.}]
suffix=''
if len(sys.argv)>1:
    configs=json.loads((here/sys.argv[1]).read_text());suffix='_'+Path(sys.argv[1]).stem
else:
    (here/'configs_lower.json').write_text(json.dumps(configs,indent=2)+'\n')
# The recorded road is NOT reconstructed. This all-free bookkeeping grid disables
# road-exit tests; the experiment measures lower-controller response/tracking only.
bitmap=here/'bookkeeping_free.bin';bitmap.write_bytes(bytes(200*200))
scenes=[]
for tag,bag in [('S1','drive_debug_20261006_014143_light'),('S2','drive_debug_20261006_014303_light')]:
    d=pickle.load((root/bag/'analysis/decoded.pkl').open('rb'))
    odom=d['/motive/vehicle/odom_map'];paths=d['/planner/lower_reference_path/with_arclength']
    start=odom[0];t0=start['t'];prior=[q for q in paths if q['t']<=t0]
    assert prior
    refs=[dict(t=0.,xy=prior[-1]['xy'].tolist(),s=prior[-1]['s'].tolist())]
    refs += [dict(t=q['t']-t0,xy=q['xy'].tolist(),s=q['s'].tolist()) for q in paths if q['t']>t0]
    duration=min(odom[-1]['t']-t0,refs[-1]['t'])
    speed=[[max(0.,q['t']-t0),float(np.linalg.norm(q['v']))] for q in odom if q['t']<=t0+duration]
    center=np.array([q['xy'] for q in odom[::5]]);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(center,axis=0),axis=1))]
    scenes.append(dict(name=f'SC3_{tag}_fixed_plans_no_road_validation',mode='replay',route=[center[0].tolist(),center[-1].tolist()],
        center=center.tolist(),arc=arc.tolist(),widths=[200.]*len(center),spur=[],bitmap=str(bitmap),origin=[0.,0.],
        resolution=2.,width=200,height=200,start=[*start['xy'],start['q'][2]],v0=speed[0][1],initial_delta=0.,
        duration=duration,references=refs,speed_profile=speed,source_bag=bag,source_start=t0))
data=dict(config_file=str(here/'baseline.yaml'),configs=configs,live_extra_configs=[],scenes=scenes,
    plants=[dict(name='tau0.74',tau=.74,delay=0,yaw_noise_deg=0),dict(name='tau0.5',tau=.5,delay=0,yaw_noise_deg=0),
        dict(name='tau1.0',tau=1.,delay=0,yaw_noise_deg=0)])
inp=here/f'input_lower_recorded{suffix}.json';inp.write_text(json.dumps(data))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
with (here/f'lower_recorded{suffix}.log').open('w') as log:
    subprocess.run([str(here.parent/'verify/sim_prod'),str(inp),str(here/f'results_lower_recorded{suffix}.jsonl')],
        env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
print('Completed',len(configs)*6,'fixed-plan lower response runs; road/branch behavior not validated.')
