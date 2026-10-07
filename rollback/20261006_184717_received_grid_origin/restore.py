from pathlib import Path
import hashlib,json
here=Path(__file__).resolve().parent
m=json.loads((here/'manifest.json').read_text());p=Path(m['target']);current=p.read_bytes()
if hashlib.sha256(current).hexdigest()!=m['after_sha256']:
    raise SystemExit('Refusing restore: configuration changed since this patch. Review and restore only the intended parameter manually.')
original=(here/'tracking_control_split.before.yaml').read_bytes()
assert hashlib.sha256(original).hexdigest()==m['before_sha256']
p.write_bytes(original)
print('Restored previous configuration; no process restart or hardware action performed.')
