#!/usr/bin/env python3
import argparse, hashlib, json, os, shutil
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--apply',action='store_true');args=parser.parse_args()
root=Path('/home/imac/ros2_ws'); backup=Path(__file__).resolve().parent
before=json.loads((backup/'before_sha256.json').read_text());after=json.loads((backup/'after_sha256.json').read_text())
for name,expected in after.items():
    p=root/name
    if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected:
        raise SystemExit('REFUSED: later change or missing file: '+name)
for name,expected in before.items():
    if hashlib.sha256((backup/'before'/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('REFUSED: backup mismatch: '+name)
if not args.apply:
    print('All hashes match. --apply restores pre-fix source/config/binary; no processes are started.')
else:
    for name in after:
        p=root/name
        if name in before:
            shutil.copy2(backup/'before'/name,p)
            if not name.startswith('build/'):
                os.utime(p,None)  # Force a future rebuild to use restored source, not cached objects.
        else:
            p.unlink()
    print('Restored. No running process or hardware was changed.')
