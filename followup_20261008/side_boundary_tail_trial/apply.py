from pathlib import Path
import json,hashlib,shutil,datetime
H=Path(__file__).resolve().parent;R=H.parents[1];hashes=json.loads((H/'baseline_sha256.json').read_text())
assert '[  PASSED  ] 9 tests.' in (H/'tests.log').read_text()
for name,sha in hashes.items():assert hashlib.sha256((R/name).read_bytes()).hexdigest()==sha,name
stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');B=R/'rollback'/f'{stamp}_side_boundary_tail';B.mkdir(parents=True)
paths=list(hashes)+['build/virtual_control/upper_planner_node']
for name in paths:
 dst=B/'before'/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(R/name,dst)
(B/'paths.json').write_text(json.dumps(paths,indent=2));(H/'rollback_path.txt').write_text(str(B)+'\n')
for name in hashes:
 s=(H/'trial'/name).read_bytes()
 if name.endswith('tracking_control_split.yaml'):s=s.replace(b'guard_side_boundary_terminal_loss: false',b'guard_side_boundary_terminal_loss: true')
 (R/name).write_bytes(s)
print('Applied source and enabled config; backup',B)
