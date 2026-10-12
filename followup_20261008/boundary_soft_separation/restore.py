"""Default: check only. --restore restores ONLY an exact copy of this candidate source."""
from pathlib import Path
import hashlib,json,sys,shutil
H=Path(__file__).resolve().parent;R=H.parents[1];rel='src/virtual_control/src/tracking_control.cpp'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((H/'baseline_sha256.json').read_text());changed=[x for x,v in manifest.items() if not (R/x).exists() or sha(R/x)!=v]
if not changed:print('운영 파일이 시작 상태와 같습니다. 복원할 변경이 없습니다.');sys.exit(0)
if changed!=[rel] or sha(R/rel)!=sha(H/'candidate'/rel):
 print('이후 변경을 발견했습니다. 덮어쓰지 않습니다:',changed);sys.exit(1)
if '--restore' not in sys.argv:
 print('이 후보와 동일한 소스 변경이 있습니다. 명시적 복원은 --restore로 실행하세요.');sys.exit(1)
shutil.copy2(H/'baseline'/rel,R/rel)
print('후보 소스만 시작 상태로 복원했습니다. 빌드/설치/프로세스 변경은 수행하지 않았습니다.')
