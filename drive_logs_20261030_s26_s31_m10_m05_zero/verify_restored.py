import hashlib, json, sys
from pathlib import Path
manifest = json.loads(Path(__file__).with_name('original_files.json').read_text())
root = Path(sys.argv[1])
for item in manifest:
    path = root / item['path']
    assert path.stat().st_size == item['bytes'], path
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    assert digest.hexdigest() == item['sha256'], path
print(f"Verified {len(manifest)} original files")
