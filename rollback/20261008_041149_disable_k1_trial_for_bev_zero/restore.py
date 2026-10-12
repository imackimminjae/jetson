from pathlib import Path
import hashlib,json
h=Path(__file__).resolve().parent
m=json.loads((h/'manifest.json').read_text());p=Path(m['target'])
if hashlib.sha256(p.read_bytes()).hexdigest()!=m['after_sha256']:
 raise SystemExit('Refusing restore: configuration changed after this edit.')
b=(h/'tracking_control_split.before.yaml').read_bytes()
assert hashlib.sha256(b).hexdigest()==m['before_sha256']
p.write_bytes(b)
print('Previous k1 trial flag restored; no process restart performed.')
