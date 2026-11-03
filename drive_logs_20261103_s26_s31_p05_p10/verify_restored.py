import hashlib, json, sys
from pathlib import Path
root = Path(sys.argv[1])
manifest = json.loads(Path(__file__).with_name('original_files.json').read_text())
for r in manifest:
    p = root / r['path']
    assert p.stat().st_size == r['bytes'], p
    h = hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda: stream.read(1024**2), b''):
            h.update(block)
    assert h.hexdigest() == r['sha256'], p
print(f'Verified {len(manifest)} original files')
