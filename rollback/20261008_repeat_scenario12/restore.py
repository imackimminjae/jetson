"""Restore only this change; refuse to overwrite subsequent edits."""
from pathlib import Path
import hashlib,json,shutil,sys
H=Path(__file__).resolve().parent;R=H.parents[1]
old=json.loads((H/'before_sha256.json').read_text());after=json.loads((H/'after_sha256.json').read_text())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
if all(sha(R/n)==old.get(n) for n in after):
 print('이미 복원된 상태입니다.');sys.exit(0)
changed=[n for n,s in after.items() if sha(R/n)!=s]
if changed:print('이후 변경을 발견했습니다. 덮어쓰지 않습니다:',changed);sys.exit(1)
if '--restore' not in sys.argv:
 print('변경 후 해시가 일치합니다. 복원하려면 --restore를 붙이세요.');sys.exit(0)
for n in after:
 if n in old:shutil.copy2(H/'before'/n,R/n)
 else:(R/n).unlink()
print('시나리오1·2 반복 실행 추가분만 복원했습니다. 실행 중 프로세스는 변경하지 않았습니다.')
