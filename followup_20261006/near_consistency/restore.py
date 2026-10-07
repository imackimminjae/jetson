"""Guarded baseline restore. Offline experiment does not change production files."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import datetime

here = Path(__file__).resolve().parent
root = here.parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--restore', action='store_true', help='Restore only with a recorded applied_sha256.json')
args = p.parse_args()
before = json.loads((here/'baseline/sha256.json').read_text())
def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
for name, expected in before.items():
    if digest(here/'baseline'/name) != expected:
        raise SystemExit('Backup mismatch: '+name)
changed = [name for name, expected in before.items() if digest(root/name) != expected]
if not changed:
    print('Baseline integrity verified; production and original harnesses unchanged. Nothing to restore.')
    raise SystemExit(0)
if not args.restore:
    raise SystemExit('Changed since snapshot (no writes): '+', '.join(changed))
applied_file = here/'applied_sha256.json'
if not applied_file.exists():
    raise SystemExit('REFUSED: no production change was applied by this experiment; preserve later edits.')
after = json.loads(applied_file.read_text())
if set(after) != set(before):
    raise SystemExit('REFUSED: incomplete applied hash manifest')
for name, expected in after.items():
    if digest(root/name) != expected:
        raise SystemExit('REFUSED: file changed after application: '+name)
archive = here/('replaced_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
for name in changed:
    saved = archive/name
    saved.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root/name,saved)
for name in changed:
    shutil.copy2(here/'baseline'/name,root/name)
print('Restored baseline; rebuild before using restored source. No processes were controlled.')
