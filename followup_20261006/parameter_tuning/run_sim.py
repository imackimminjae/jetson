from pathlib import Path
import subprocess,os,json,sys
here=Path(__file__).resolve().parent;root=here.parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
group=sys.argv[1]
configs=json.loads((here/(sys.argv[2] if len(sys.argv)>2 else 'configs.json')).read_text())
data=json.loads((here.parent/'verify'/f'input_{group}_prod.json').read_text())
data['configs']=configs;data['live_extra_configs']=[]
data['config_file']=str(here/'baseline.yaml')
suffix=('_'+Path(sys.argv[2]).stem) if len(sys.argv)>2 else ('_screen' if group=='synthetic' else '')
inp=here/f'input_{group}{suffix}.json';inp.write_text(json.dumps(data))
with (here/f'sim_{group}{suffix}.log').open('w') as log:
    subprocess.run([str(here.parent/'verify/sim_prod'),str(inp),str(here/f'results_{group}{suffix}.jsonl')],
        env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
print('Completed',group,len(configs),'configs',flush=True)
