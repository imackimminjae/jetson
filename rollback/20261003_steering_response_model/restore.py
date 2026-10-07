#!/usr/bin/env python3
"""Restore this trial's exact original configuration without overwriting later edits."""
import hashlib
import json
from pathlib import Path

folder = Path(__file__).resolve().parent
change = json.loads((folder / "change.json").read_text())
target = Path(change["source"])
current = hashlib.sha256(target.read_bytes()).hexdigest()
if current == change["before_sha256"]:
    print("Already restored: steering response model tau = 0.05 s")
elif current == change["applied_sha256"]:
    original = (folder / "tracking_control_split.before.yaml").read_bytes()
    if hashlib.sha256(original).hexdigest() != change["before_sha256"]:
        raise SystemExit("Backup checksum mismatch; no file changed.")
    target.write_bytes(original)
    print("Restored tau = 0.05 s. Restart the controller to load it.")
else:
    raise SystemExit("Configuration changed after this trial; automatic restore refused to preserve later edits.")
