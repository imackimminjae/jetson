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
It initializes B immediately. With the default
`preview_interval_reference_growth_rate_mps: 0.0`, an existing B is retained when
no candidate exists or candidate B > 1.5*B; otherwise B is replaced immediately.
A positive growth-rate setting keeps immediate decreases and limits increases
per configured planning period, including gradual recovery beyond the legacy
1.5*B growth gate. See [the optional B growth limit](interval_reference_growth.md).
B is the only retained algorithm state. Missing or stale required grid data
leaves B unchanged.

Observed intervals are ON iff B exists and L <= 1.5*B. ON keeps the central
`(2*soft_ratio - 1)` fraction: the split profile and node default use 0.7,
retaining 40%. The ratio is configurable in `(0.5, 1.0]`.
OFF insets each endpoint by min(2.0 m, 0.45*L) in the split profile,
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

## Additional sections used only to calculate B

`preview_interval_b_sample_spacing_m` defaults to `0.0` (disabled). A positive
value adds sections at that physical arc-length spacing along the existing
usable reference. The prepared full-scale trial configs use `0.6` m. Existing
prediction stages are retained; intermediate points coinciding with an existing
stage are omitted. Position is linearly interpolated along the existing polyline,
and heading follows the shortest wrapped interpolation of its recorded/reference
headings. There is no reference extrapolation, timing resampling, or new solver step.
Spacing must be finite and either zero or at least `preview_line_sample_m`.
Pathological requests exceeding 4096 probes are left without extra probes.

The exact same observed-grid traversal is used for original stages and B-only
sections. Unknown/outside termination is not an observed boundary. Extra sections
cannot use nominal fallback, and the original unique-reference-containment and
both-boundaries-observed eligibility rules remain in effect. Eligible sections
compete with original stages for the minimum B candidate. The existing B growth
policy, soft ratio, and interval-treatment policy then apply once per planning
cycle. This option introduces no persistent state.

Only B receives the added observations. MIQP reference points, horizon, raw lane
intervals, candidate count, heading costs and vehicle constraints are unchanged
by the sampling operation. A changed B does change processed lane bounds, and
can consequently alter branch continuity/selection or later closed-loop motion.
This is not a guarantee that candidate selection or the published path is unchanged.

Existing interval-info rows still describe only original prediction stages. The
seven-column B state message is unchanged. Its `candidate_step` is a **B sample
index**: original stages retain indices `0..requested_horizon`; extra samples
start at `requested_horizon+1`. Missing candidate remains `-1`. Original samples
are inspected first and win exact length ties. Indices beyond the horizon must
not be interpreted as MIQP prediction stages.

`/debug/upper_interval_b_samples` records extra sections in 14-column rows:

    stamp,sample,parent_step,s,x_body,y_body,heading,candidate,raw,
    start_observed,end_observed,inside,eligible,selected

`stamp` is shared with the accompanying B state diagnostic; `parent_step` is the
preceding original stage and `s` is reference arc length. Multiple interval rows
may share a sample index. A section with no observed free runs has no rows.
`selected` denotes the winning minimum candidate, even if the existing B update
gate rejects its increase. The shared essential recording profile includes this
new diagnostic topic; it is not a required-ready topic and changes no stop guard.

Implementation and saved-input checks are recorded under
`followup_20261016/dense_b_implementation/`. Frozen-input geometry checks do not
establish actual body-yaw improvement or safety on a full vehicle footprint.
