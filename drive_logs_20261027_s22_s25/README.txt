Scenario 22 / 23 / 24 / 25 successful driving recordings, 2026-10-27

17 completed attempts: scenario22 5, scenario23 5, scenario24 2, scenario25 5.
Scenario24 includes only the two successful corrected west-passage runs:
  2026-10-27 07:13:11 batch: 71.299 seconds, completed.
  2026-10-27 07:17:09 batch: 68.418 seconds, completed.
Times are Asia/Seoul. Failed scenario24 attempts, aborted startups, continuous
MAVLink telemetry and analyses containing failed scenario24 records are excluded.
Original MCAPs, successful-session settings/logs and matching analyses are kept.

Verify and restore from this directory:
  sha256sum -c SHA256SUMS
  mkdir -p restored
  cat scenario_logs.tar.zst.part-* | zstd -d -c | tar -xf - -C restored

original_files.json lists original file sizes and SHA-256 values.
sessions.csv indexes only the 17 completed attempts.
UPLOAD.json records the exact selection and archive parts.
route_versions.json records the two scenario24 route hashes. The final route CSV
is included; the first successful run's exact CSV snapshot is unavailable, so its
hash and original recording are preserved without substituting the final CSV.

Archive paths preserve workspace-relative names. Local source logs are untouched.
