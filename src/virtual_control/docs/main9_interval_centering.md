# Main9 interval centering in the ROS2 planner

The active `upper_planner_node` ports the main9 rules into C++. It does not call
MATLAB. The existing geometry, nominal/correction inputs, heading target,
heading activation window, normalization, and position/input costs remain intact.
Only `lambda_relative_turn` changes to 0.9 in the default configuration.

## State and decisions

`interval_centering::update(previous, observations, settings)` is a pure state
transition. `extractPreviewIntervals` receives the previous state explicitly and
returns the next state in `PreviewConstraintData::interval_update`. The node owns
one state initialized at startup and passes it between planning cycles, including
cycles whose solver fails. No global or persistent function state is used.

Every requested preview stage is extracted before the update, including stages
beyond a missing interval that shortens the MIQP horizon. Near pairs are tested
in order (k=0,1), then (k=1,2). Each stage must have exactly one non-fallback
interval with two known blocked boundary samples, a reference point inside it,
and positive finite length. Unknown samples and map-edge clipping do not count
as observed boundaries. A pair qualifies when max/min <= 1.2; its mean is the
median of the two lengths.

The first candidate initializes B. Afterwards any genuinely observed multiple
intervals anywhere in the preview freeze B and clear confirmation history.
Missing near candidates also freeze B and clear history. The rolling window of
three consecutive valid candidates must have max/min <= 1.2. Its median is the
update target, clamped to [B/1.05, B*1.05]. A successful update retains the rolling
window; each subsequent valid cycle uses the latest three candidates. Missing or
stale required grid data clears confirmation without discarding B.

Observed intervals are ON iff B exists and L <= 1.5*B. ON keeps the central 40%
using the fixed soft ratio 0.7. OFF insets each endpoint by min(2.0 m, 0.45*L),
including when B is unavailable. OFF can therefore be narrower than ON on short
intervals. B is an observed length scale, not a physical road-width estimate.

Nominal fallback remains enabled with width 4.5 m and an independent 9 m
centering threshold. Fallback never supplies a B candidate or counts as an
observed branch. The existing rule forbidding fallback at a known blocked
reference cell is retained.

## Diagnostics

`/debug/upper_constraint_intervals` retains its existing seven-column format and
covers the solver's usable prefix. Treatment 0 means ON, treatment 1 means OFF.

`/debug/upper_interval_info` contains 12 columns per raw interval across the full
requested preview. The layout labels name the columns:

| Column | Meaning |
| --- | --- |
| 0, 1 | k, candidate index |
| 2, 3 | raw and processed lengths, metres |
| 4 | ON (1) / OFF (0) |
| 5, 6 | start/end boundary observed |
| 7 | observed interval clipped by map extent or unknown samples |
| 8, 9 | nominal fallback, reference inside |
| 10, 11 | B (0 if unavailable), decision threshold (NaN if unavailable) |

`/debug/upper_interval_centering_state` publishes one 12-element record per
preview: ROS timestamp, previous B, current B, observed threshold, candidate,
pair start k, confirmation count, confirmation minimum, confirmation maximum,
update target, update reason, observed-multiple flag. Unavailable candidate,
threshold, history bounds and update target are NaN. Pair start is -1 if absent.

Update reasons: 0=no candidate, 1=initialized, 2=observed multiple intervals,
3=confirming, 4=candidates disagree, 5=updated. Verbose text diagnostics are
available through `preview_interval_debug`.

## Verification

`test_interval_centering_state` covers near-pair eligibility, initialization,
branch/gap holds, three-candidate confirmation, both update bounds and fixed
ON/OFF/fallback treatment. The opt-in ROS integration test uses a separate domain
and synthetic grids, and starts only the actuator-free upper planner:

```bash
source install/setup.bash
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 INTERVAL_CENTERING_ROS_TEST=1 \
  python3 -m pytest -q src/virtual_control/test/test_interval_centering_ros.py
```
