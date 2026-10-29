Scenario 22 / 23 / 24 / 25 HILS recordings, 2026-10-30 (Asia/Seoul)

12 completed attempts: each scenario 3 times, waypoint bias 0 degrees.
Lower: 3-state, tau=0.74 s, pure delay=0 s. Upper: DAQP BnB for every integer choice.
S22 starts 2.4 m closer to the observed road center; S24 runs south to north.
The pre-edit interrupted batch is not included. Original local files are unchanged.

Verify and restore from this directory:
  sha256sum -c SHA256SUMS
  mkdir -p restored
  cat scenario_logs.tar.zst.part-* | zstd -d -c | tar -xf - -C restored

Archive paths are workspace-relative, including the full selected batch, source
snapshot, diagnostics, exact frozen routes, configuration hashes, and analyses.
original_files.json lists each archived file's size and SHA-256.
sessions.csv and route_versions.json index all twelve completed attempts/routes.
UPLOAD.json records the exact selection and all archive parts.

After restore, verify original file hashes (run in this directory):
  python3 verify_restored.py restored

The live result contains 41 BnB numerical failures in 10,502 BnB calls.
All twelve drives completed; this does not establish global optimality of BnB.
See README_20261030.md and the included solver verification records.
