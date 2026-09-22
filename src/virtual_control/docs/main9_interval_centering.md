# Main9 interval centering in the ROS2 planner

The active `upper_planner_node` ports the main9 rules into C++. It does not call
MATLAB. The existing geometry, nominal/correction inputs, heading target,
heading activation window, normalization, and position/input costs remain intact.

## State and decisions

`SdMapUpperPlannerNode::updateIntervalCentering` and `intervalTreatment` are
private helpers implemented inside `tracking_control.cpp`. There is no separate
centering header, executable, or ROS node. The update is a pure state transition. `extractPreviewIntervals` receives the previous state explicitly and
returns the next state in `PreviewConstraintData::interval_update`. The node owns
one state initialized at startup and passes it between planning cycles, including
cycles whose solver fails. No global or persistent function state is used.

Every requested preview stage k=0..N is extracted before the update, including
stages beyond a missing interval that shortens the MIQP horizon. At each stage,
exactly one interval must contain the current reference point. That interval
qualifies only if it is non-fallback, has both boundaries observed, and has a
positive finite original length. Unknown samples and map-edge clipping do not
count as observed boundaries. Other branches do not supply candidates or block
updates. Multiple intervals containing the reference make that stage ambiguous,
even when one of those intervals is unreliable.

The shortest eligible original length across the entire preview is candidate B.
It initializes B immediately. An existing B is retained when no candidate exists
or candidate B > 1.5*B; otherwise B is replaced immediately. B is the only retained
algorithm state. Missing or stale required grid data leaves B unchanged.

Observed intervals are ON iff B exists and L <= 1.5*B. ON keeps the central 40%
using the fixed soft ratio 0.7. OFF insets each endpoint by min(2.0 m, 0.45*L),
including when B is unavailable. OFF can therefore be narrower than ON on short
intervals. B is an observed length scale, not a physical road-width estimate.

Nominal fallback remains enabled with width 4.5 m and an independent 9 m
centering threshold. Fallback never supplies a B candidate. The existing rule
forbidding fallback at a known blocked reference cell is retained.

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

`/debug/upper_interval_centering_state` publishes one seven-element record per
preview: ROS timestamp, previous B, applied B, 1.5*B, candidate B, candidate step k,
candidate interval index. Unavailable candidate and threshold are NaN; both
candidate indices are -1 if absent. Layout labels identify these columns. This
replaces the previous 12-element state record; the interval detail and legacy
seven-column interval topics retain their formats.

Verbose centering logs (`preview_interval_debug`) report only previous/candidate/
applied B, 1.5*B, the selected step and interval, and each interval's original
length and ON/OFF decision.

## Verification

`test_interval_centering_state` covers full-preview minimum selection, reference
branch membership, ambiguity/clipping/fallback rejection, immediate decreases,
held excessive increases, and fixed ON/OFF/fallback treatment. The opt-in ROS
integration test checks synthetic grids and the diagnostic schema on a separate
domain, starting only the actuator-free upper planner:

```bash
source install/setup.bash
ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 INTERVAL_CENTERING_ROS_TEST=1 \
  python3 -m pytest -q src/virtual_control/test/test_interval_centering_ros.py
```
