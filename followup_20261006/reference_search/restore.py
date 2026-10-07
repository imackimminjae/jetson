from pathlib import Path
import hashlib,json
p=Path(__file__).resolve().parent; r=p.parents[1]
for f,h in json.loads((p/"baseline/sha256.json").read_text()).items():
 assert hashlib.sha256((p/"baseline"/f).read_bytes()).hexdigest()==h, "backup mismatch: "+f
 assert hashlib.sha256((r/f).read_bytes()).hexdigest()==h, "later edit detected (preserved): "+f
print("Original files match backup; no operational changes to restore. No writes.")
