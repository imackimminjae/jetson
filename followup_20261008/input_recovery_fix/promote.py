#!/usr/bin/env python3
"""Manual, hash-checked promotion/rollback. Never start/stop ROS or hardware."""
import argparse,hashlib,json,os,stat,tempfile
from pathlib import Path
H=Path(__file__).resolve().parent;ROOT=H.parents[1];DEST=ROOT/'src/virtual_control/scripts'
FILES=('knu_batch_runner.py','drive_debug_profile.py','repeat_drive_session.py')
NODES={'knu_batch_runner.py','repeat_drive_session.py','upper_planner_node','lower_tracking_mpc_node','px4_odom_map_bridge','px4_ekf_bridge.py'}
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def active_nodes():
 out=[]
 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:args=p.read_bytes().decode(errors='replace').split('\0');matches={Path(a).name for a in args if a}&NODES
  except (OSError,ValueError):continue
  if matches:out.append((p.parent.name,sorted(matches)))
 return out
def atomic_write(p,content):
 mode=stat.S_IMODE(p.stat().st_mode)
 fd,name=tempfile.mkstemp(prefix='.'+p.name+'.',dir=p.parent)
 try:
  with os.fdopen(fd,'wb') as f:f.write(content);f.flush();os.fsync(f.fileno())
  os.chmod(name,mode);os.replace(name,p)
 finally:
  if os.path.exists(name):os.unlink(name)
def main():
 parser=argparse.ArgumentParser();g=parser.add_mutually_exclusive_group();g.add_argument('--check',action='store_true');g.add_argument('--apply',action='store_true');g.add_argument('--rollback',action='store_true');a=parser.parse_args()
 manifest=json.loads((H/'package_sha256.json').read_text())
 for rel,sha in manifest.items():
  if digest(H/rel)!=sha:raise SystemExit('Package changed; revalidate before applying: '+rel)
 if not a.apply and not a.rollback:
  for n in FILES:print(n,'baseline' if digest(DEST/n)==digest(H/'baseline'/n) else 'candidate' if digest(DEST/n)==digest(H/'candidate'/n) else 'CHANGED')
  return
 live=active_nodes()
 if live:raise SystemExit('Refusing to replace files while related processes are active: '+str(live))
 expected='candidate' if a.rollback else 'baseline';replacement='baseline' if a.rollback else 'candidate'
 for n in FILES:
  if digest(DEST/n)!=digest(H/expected/n):raise SystemExit('Current file changed; preserving it: '+str(DEST/n))
 changed=[]
 try:
  for n in FILES:atomic_write(DEST/n,(H/replacement/n).read_bytes());changed.append(n)
 except BaseException:
  for n in reversed(changed):atomic_write(DEST/n,(H/expected/n).read_bytes())
  raise
 print('Replaced three script files only; no processes started. Mode:',replacement)
if __name__=='__main__':main()
