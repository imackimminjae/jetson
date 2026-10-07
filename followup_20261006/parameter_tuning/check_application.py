from pathlib import Path
import hashlib,json,shutil,tempfile,subprocess,os
here=Path(__file__).resolve().parent;root=here.parents[1];rb=root/'rollback/20261006_lower_rd150'
applied=root/'src/virtual_control/config/tracking_control_split.yaml'
with tempfile.TemporaryDirectory(prefix='rd150_restore_',dir='/tmp') as temp:
    fake=Path(temp);folder=fake/'rollback/20261006_lower_rd150';shutil.copytree(rb,folder)
    target=fake/'src/virtual_control/config/tracking_control_split.yaml';target.parent.mkdir(parents=True)
    target.write_bytes(applied.read_bytes())
    def run(*args):return subprocess.run(['python3',str(folder/'restore.py'),*args],capture_output=True,text=True)
    assert run().returncode==0
    changed=target.read_bytes()+b'\n# subsequent user edit\n';target.write_bytes(changed)
    assert run('--restore').returncode!=0 and target.read_bytes()==changed
    target.write_bytes(applied.read_bytes());assert run('--restore').returncode==0
    assert target.read_bytes()==(folder/'tracking_control_split.yaml.before').read_bytes()
    assert run('--restore').returncode==0
    target.write_bytes(applied.read_bytes());saved=(folder/'tracking_control_split.yaml.before').read_bytes()
    (folder/'tracking_control_split.yaml.before').write_bytes(saved+b'\n')
    assert run('--restore').returncode!=0 and target.read_bytes()==applied.read_bytes()
print('Five restore-guard cases passed in an isolated temporary directory.')
data=json.loads((here/'input_synthetic_configs_lower_small.json').read_text())
data['config_file']=str(root/'install/virtual_control/share/virtual_control/config/tracking_control_split.yaml')
data['configs']=[dict(name='installed_rd150')]
inp=here/'input_installed_check.json';inp.write_text(json.dumps(data))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(here/'ros_logs'))
with (here/'installed_check.log').open('w') as log:
    subprocess.run([str(here.parent/'verify/sim_prod'),str(inp),str(here/'results_installed_check.jsonl')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
import numpy as np
expected=[json.loads(l) for l in (here/'results_synthetic_configs_lower_small.jsonl').read_text().splitlines()]
actual=[json.loads(l) for l in (here/'results_installed_check.jsonl').read_text().splitlines()]
assert len(actual)==len(expected)==2
for a,b in zip(actual,expected):
    assert a['scene']==b['scene'] and a['outcome']==b['outcome'] and len(a['states'])==len(b['states'])
    assert all(np.allclose([x[k] for k in ['x','y','yaw','command']],[y[k] for k in ['x','y','yaw','command']],atol=1e-10,rtol=0) for x,y in zip(a['states'],b['states']))
for name,h in json.loads((rb/'unchanged_sha256.json').read_text()).items():
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==h,name
(here/'application_checks.json').write_text(json.dumps(dict(restore_guard_cases=5,installed_yaml_matches_offline_override=2,
    controller_sources_and_binaries_unchanged=True,hardware_actions=False),indent=2)+'\n')
print('Installed YAML reproduces both nominal synthetic rd150 trajectories; sources/binaries unchanged.')
