"""Check only by default; restore task-start files only if later edits are absent."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/imac/ros2_ws')
parser = argparse.ArgumentParser()
parser.add_argument('--apply', action='store_true')
args = parser.parse_args()
manifest = json.loads((HERE/'restore_manifest.json').read_text())
for rel, checks in manifest.items():
    current, backup = ROOT/rel, HERE/'before'/rel
    if hashlib.sha256(current.read_bytes()).hexdigest() != checks['after_sha256']:
        raise SystemExit(f'이후 변경이 있어 복원 중단: {current}')
    if hashlib.sha256(backup.read_bytes()).hexdigest() != checks['before_sha256']:
        raise SystemExit(f'백업 검증 실패: {backup}')
if args.apply:
    for rel in manifest:
        shutil.copyfile(HERE/'before'/rel, ROOT/rel)
    print('작업 직전 코드 복원 완료')
else:
    print('복원 가능 확인 완료. 파일 변경 없음.')
