S26, waypoint bias -20 degrees, 100 attempts.

One attempt per recording for new runs; imported earlier completed attempts are indexed by token. Driving failures are retained. System interruptions are excluded and retried. Three-state actuator model tau=0.74 s, DAQP branch-and-bound; pose is not biased.

Restore: `cat logs/logs.tar.zst.part-* | zstd -d | tar -xf -`
