from pathlib import Path
import json,hashlib,os,tempfile
B=Path(__file__).resolve().parent
R=Path('/home/imac/ros2_ws')
expected=json.loads((B/'after_sha256.json').read_text())
for name,sha in expected.items():
 p=R/name
 if not p.exists() or hashlib.sha256(p.read_bytes()).hexdigest()!=sha:
  raise SystemExit('Refusing to overwrite later changes: '+name)
for name in expected:
 p=R/name;old=B/'before'/name
 fd,tmp=tempfile.mkstemp(prefix='.side_tail_restore_',dir=p.parent)
 with os.fdopen(fd,'wb') as f:f.write(old.read_bytes())
 os.chmod(tmp,old.stat().st_mode & 0o777)
 os.replace(tmp,p)
print('Restored source/config/planner binary. No running process was restarted.')
