"""Restore this parameter edit only; refuse to overwrite subsequent YAML edits."""
from pathlib import Path
import argparse,datetime,hashlib,shutil
here=Path(__file__).resolve().parent
target=here.parents[1]/'src/virtual_control/config/tracking_control_split.yaml'
backup=here/'tracking_control_split.yaml.before'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--restore',action='store_true')
args=parser.parse_args()
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
before=(here/'before_sha256.txt').read_text().split()[0]
if digest(backup)!=before:raise SystemExit('REFUSED: backup hash mismatch.')
current=digest(target)
if current==before:
    print('Original YAML already present; no writes. No restart or build required for the file restore.')
    raise SystemExit(0)
after=here/'after_sha256.txt'
if not after.exists() or current!=after.read_text().split()[0]:
    raise SystemExit('REFUSED: YAML differs from the recorded application; preserving later edits.')
if not args.restore:
    print('Backup and applied YAML verified. Use --restore to restore lower_rd_steering_rate=120.')
    raise SystemExit(0)
archive=here/('replaced_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.yaml')
shutil.copy2(target,archive)
shutil.copy2(backup,target)
print('Restored the previous YAML. Effective at next normal node startup; no process/hardware actions.')
