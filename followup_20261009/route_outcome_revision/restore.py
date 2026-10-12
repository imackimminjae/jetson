"""Restore only this revision's edits, preserving the pre-existing dirty tree."""
from pathlib import Path
import hashlib,json,sys,shutil

O=Path(__file__).resolve().parent
R=O.parents[1]
items=json.loads((O/'restore_manifest.json').read_text())
for name,info in items.items():
    p=R/name
    if hashlib.sha256(p.read_bytes()).hexdigest()!=info['after_sha256']:
        raise SystemExit(f'중단: 이후 변경이 있는 파일은 덮어쓰지 않습니다: {p}')
if sys.argv[1:] == ['--apply']:
    for name,info in items.items():shutil.copy2(O/info['backup'],R/name)
    print('이번 판정 변경만 복원했습니다. 이전 미커밋 변경과 로그/검증 자료는 보존했습니다.')
elif not sys.argv[1:]:
    print('복원 가능한 파일 확인 완료. 실제 복원은 --apply 옵션으로 실행합니다.')
else:
    raise SystemExit('사용법: python3 restore.py [--apply]')
