from pathlib import Path
import shutil,hashlib,json
H=Path(__file__).resolve().parent;R=H.parents[1]
files=['src/virtual_control/src/tracking_control.cpp','src/virtual_control/include/virtual_control/tracking_control.hpp','src/virtual_control/config/tracking_control_split.yaml','src/virtual_control/test/test_lower_path_generation.cpp']
hashes={}
for name in files:
 src=R/name;dst=H/'baseline'/name;dst.parent.mkdir(parents=True,exist_ok=True)
 if not dst.exists():shutil.copy2(src,dst)
 assert src.read_bytes()==dst.read_bytes(),name
 hashes[name]=hashlib.sha256(src.read_bytes()).hexdigest()
(H/'baseline_sha256.json').write_text(json.dumps(hashes,indent=2))
for name in files:
 dst=H/'trial'/name;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(H/'baseline'/name,dst)
