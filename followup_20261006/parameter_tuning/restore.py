"""Verify experiment backups, then delegate the guarded YAML-only restore."""
from pathlib import Path
import hashlib,json,subprocess,sys
p=Path(__file__).resolve().parent;root=p.parents[1]
for name,h in json.loads((p/'baseline/sha256.json').read_text()).items():
    assert hashlib.sha256((p/'baseline'/name).read_bytes()).hexdigest()==h,'backup mismatch: '+name
    if name!='src/virtual_control/config/tracking_control_split.yaml':
        assert hashlib.sha256((root/name).read_bytes()).hexdigest()==h,'later edit (preserved): '+name
raise SystemExit(subprocess.call([sys.executable,str(root/'rollback/20261006_lower_rd150/restore.py'),*sys.argv[1:]]))
