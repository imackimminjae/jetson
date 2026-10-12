#!/usr/bin/env python3
"""Check every post-change hash before restoring; never overwrite later work."""
import argparse, hashlib, json, shutil
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('--apply',action='store_true')
args=parser.parse_args()
root=Path('/home/imac/ros2_ws'); backup=Path(__file__).resolve().parent
before=json.loads((backup/'before_sha256.json').read_text())
after=json.loads((backup/'after_sha256.json').read_text())
changed=[]
for name,expected in after.items():
    path=root/name
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
        changed.append(name)
if changed:
    raise SystemExit('REFUSED: files changed after setup: '+', '.join(changed))
for name,expected in before.items():
    saved=backup/'before'/name
    if hashlib.sha256(saved.read_bytes()).hexdigest()!=expected:
        raise SystemExit('REFUSED: backup hash mismatch: '+name)
if not args.apply:
    print('All hashes match. --apply restores old files and removes new setup/shortcut files.')
else:
    for name in after:
        path=root/name
        if name in before:
            shutil.copy2(backup/'before'/name,path)
        else:
            path.unlink()
    print('Restored. No running processes or hardware were touched.')
