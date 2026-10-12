from pathlib import Path
import json,gzip,os,subprocess,yaml
H=Path(__file__).resolve().parent;R=H.parents[1];p=yaml.safe_load((H/'baseline/src/virtual_control/config/tracking_control_split.yaml').read_text())['upper_planner_node']['ros__parameters'];cfg=H/'current_config.yaml';cfg.write_text(yaml.safe_dump({'upper_planner_node':{'ros__parameters':p}}))
specs=[dict(name=label+'_'+mode,fixed_ref=mode=='fixed',fixed_B=mode=='fixed',fixed_wp=True,guard=True,**({'edge_policy':'gated_skip'} if label=='candidate' else {})) for label in ['base','candidate'] for mode in ['fixed','carried']];sp=H/'specs_current.json';sp.write_text(json.dumps(specs,indent=2));env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
for a in json.loads((H/'manifest_history.json').read_text()):
 out=H/f'current_{a["tag"]}.json'
 with (H/f'current_{a["tag"]}.log').open('w') as log:subprocess.run([str(H/'refine_replay'),a['input'],a['grids'],str(cfg),str(out),str(sp)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
 out.unlink();print(a['tag'],flush=True)
