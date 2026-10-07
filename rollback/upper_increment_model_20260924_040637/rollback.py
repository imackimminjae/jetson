#!/usr/bin/env python3
"""Restore the exact pre-experiment sources, profile and upper executable."""
import argparse, hashlib, json, os, shutil, signal, time
from pathlib import Path
parser=argparse.ArgumentParser(); parser.add_argument('--check', action='store_true'); args=parser.parse_args()
here=Path(__file__).resolve().parent; meta=json.loads((here/'manifest.json').read_text()); root=Path(meta['workspace'])
for item in meta['files']:
    source=here/'before'/item['path']
    assert hashlib.sha256(source.read_bytes()).hexdigest()==item['sha256'], 'Backup checksum mismatch: '+item['path']
if args.check:
    print('Backup checksums OK; no processes or files changed.'); raise SystemExit(0)
# Stop only this workspace's upper/lower controllers; keep MAVProxy and bridges.
executables={str(root/'build/virtual_control'/name) for name in ('upper_planner_node','lower_tracking_mpc_node')}
pids=[]
for directory in Path('/proc').iterdir():
    if not directory.name.isdigit(): continue
    try:
        if str((directory/'exe').resolve()) in executables: pids.append(int(directory.name))
    except (OSError, RuntimeError): pass
for pid in pids:
    try: os.kill(pid,signal.SIGINT)
    except ProcessLookupError: pass
until=time.monotonic()+8
while any(Path('/proc',str(pid)).exists() for pid in pids) and time.monotonic()<until: time.sleep(.1)
if any(Path('/proc',str(pid)).exists() for pid in pids):
    raise SystemExit('A controller has not stopped. Stop it, then run rollback again; files unchanged.')
# Preserve the replaced state too, including any edits made after this experiment.
archive=here/('displaced_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
for item in meta['files']:
    target=root/item['path']; saved=archive/item['path']; saved.parent.mkdir(parents=True,exist_ok=True)
    if target.exists(): shutil.copy2(target,saved)
    temporary=target.with_name(target.name+'.rollback_tmp_'+str(os.getpid()))
    shutil.copy2(here/'before'/item['path'],temporary); os.replace(temporary,target)
    # Force a future build to rebuild restored sources instead of reusing new objects.
    if item['path'].startswith('src/'): os.utime(target,None)
    assert hashlib.sha256(target.read_bytes()).hexdigest()==item['sha256']
print('Rolled back sources, profile and upper binary. Controllers remain stopped; restart normally.')
print('Replaced state saved in',archive)
