"""User-invoked rollback only; refuse to overwrite subsequent edits."""
from pathlib import Path
import hashlib
import json
import shutil

root = Path('/home/imac/ros2_ws')
backup = root/'rollback/20261009_all12_repeat5'
manifest = json.loads((Path(__file__).parent/'changed_files.json').read_text())
for row in manifest:
    path = root/row['path']
    if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
        raise SystemExit(f'복원 보류: 이후 변경 여부를 확인하세요: {path}')
for row in manifest:
    path = root/row['path']
    if row['backup_exists']:
        shutil.copy2(backup/row['path'], path)
    else:
        path.unlink()
# Ensure a subsequent target build cannot reuse objects from the newer anchors.
(root/'src/virtual_control/src/px4_odom_map_transform.cpp').touch()
print('백업 복원 완료. 사용법의 bridge 재빌드 명령을 실행하세요.')
