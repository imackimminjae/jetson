"""Verify backups; restore only the exact recorded application, preserving later edits."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import shutil

here = Path(__file__).resolve().parent
root = here.parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--restore', action='store_true')
args = parser.parse_args()


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


before = json.loads((here / 'before_sha256.json').read_text())
for name, expected in before.items():
    if digest(here / 'before' / name) != expected:
        raise SystemExit('REFUSED: backup hash mismatch: ' + name)
changed = [name for name, value in before.items() if digest(root / name) != value]
if not changed:
    print('Backups verified; source/config/test files still match the baseline. No writes.')
    raise SystemExit(0)
applied = here / 'applied_sha256.json'
if not applied.is_file():
    raise SystemExit('REFUSED: application manifest missing; preserve current files.')
after = json.loads(applied.read_text())
if set(after) != set(before):
    raise SystemExit('REFUSED: incomplete application manifest.')
for name, expected in after.items():
    if digest(root / name) != expected:
        raise SystemExit('REFUSED: later change detected: ' + name)
if not args.restore:
    print('Backups and applied hashes verified. Use --restore to restore source/config/tests.')
    raise SystemExit(0)
archive = here / ('replaced_' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
for name in changed:
    target = archive / name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / name, target)
for name in changed:
    shutil.copy2(here / 'before' / name, root / name)
print('Restored recorded source/config/test files. Rebuild virtual_control before next use.')
print('No executable replacement, process restart, or hardware action was performed.')
