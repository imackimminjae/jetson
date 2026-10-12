from pathlib import Path
import json,gzip,os,subprocess,yaml
H=Path(__file__).resolve().parent;R=H.parents[1];P=R/'followup_20261008/boundary_soft_separation'
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
def run(tag,ip,gp,cfg,specs):
    sp=H/(tag+'_specs.json');sp.write_text(json.dumps(specs))
    out=H/(tag+'.json');gp=Path(gp)
    cmd=[str(H/'upper_replay'),str(ip),'/dev/stdin' if gp.suffix=='.gz' else str(gp),str(cfg),str(out),str(sp)]
    with (H/(tag+'.log')).open('w') as f:subprocess.run(cmd,input=gzip.decompress(gp.read_bytes()) if gp.suffix=='.gz' else None,env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
    with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
    out.unlink();print(tag,flush=True)
base=dict(fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True)
ab=[dict(base,name='soft_k4',soft_mask=16),dict(base,name='soft_k5',soft_mask=32),dict(base,name='soft_k6',soft_mask=64),dict(base,name='soft_k456',soft_mask=112)]
for n in [1,3,5,6,8,10]:run(f'ablation_{n}',H/f'input_{n}.json',H/'grids.bin.gz',H/'config.yaml',ab)
manifest=json.loads((P/'manifest_batch.json').read_text())
for a in manifest:a['tag']='reg_sc3_'+a['tag']
Q=R/'followup_20261008/scenario1_detailed_review'
for n in [1,4,10]:manifest.append(dict(tag=f'reg_old_sc1_{n}',input=str(Q/f'input_{n}.json'),grids=str(Q/'grids.bin.gz'),config=str(Q/'config.yaml')))
for a in json.loads((P/'manifest_history.json').read_text()):
    if a['tag'] in ['20261006_050627','20261006_050536','20261006_014143','20261006_014303']:
        a['tag']='reg_'+a['tag'];manifest.append(a)
(H/'regression_manifest.json').write_text(json.dumps(manifest,indent=2))
for a in manifest:
    cfg=yaml.safe_load(Path(a['config']).read_text())
    base['guard']=cfg['upper_planner_node']['ros__parameters'].get('guard_side_boundary_terminal_loss',False)
    specs=[dict(base,name='fixed'),dict(base,name='B7_4',force_B=7.4),dict(base,name='B14',force_B=14.0),dict(base,name='soft_k1',soft_mask=2),dict(base,name='soft_k123',soft_mask=14),dict(base,name='soft_all',soft_mask=126)]
    run(a['tag'],a['input'],a['grids'],a['config'],specs)
