"""Check restoration by default; --apply restores only matching changed files."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

parser=argparse.ArgumentParser()
parser.add_argument('--apply',action='store_true')
args=parser.parse_args()
folder=Path(__file__).resolve().parent
manifest=json.loads((folder/'restore_manifest.json').read_text())
for item in manifest:
    target=Path(item['target'])
    digest=hashlib.sha256(target.read_bytes()).hexdigest()
    if digest not in [item['before_sha256'],item['after_sha256']]:
        raise SystemExit(f'후속 변경이 있어 덮어쓰지 않습니다: {target}')
    backup=folder/item['backup']
    if hashlib.sha256(backup.read_bytes()).hexdigest()!=item['before_sha256']:
        raise SystemExit(f'백업 해시가 다릅니다: {backup}')
for item in manifest:
    target=Path(item['target'])
    if args.apply:
        shutil.copy2(folder/item['backup'],target)
        print(f'복원 완료: {target}')
    else:
        print(f'복원 가능(검사만 수행): {target}')
