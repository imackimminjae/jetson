from pathlib import Path
import json,hashlib,shutil,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1]
files=list((R/'src/virtual_control').rglob('*'))+[R/'run_scenario3_10.sh',R/'시나리오3_10회_사용법.txt']
hashes={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file() and '__pycache__' not in str(p)}
assert not (H/'baseline_sha256.json').exists()
(H/'baseline_sha256.json').write_text(json.dumps(hashes,indent=2))
(H/'git_status_before.txt').write_text(subprocess.check_output(['git','status','--short'],cwd=R,text=True))
for rel in ['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp','src/virtual_control/config/tracking_control_split.yaml','src/virtual_control/config/scenario3_repeat.yaml']:
 p=H/'baseline'/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(R/rel,p)
for rel in ['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp']:
 p=H/'trial'/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(R/rel,p)
