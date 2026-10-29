# Branch step for turn guidance

The branch step is the first predicted section `k=2..N` with `Nck[k] >= 2`.
The measured `k=0` state and the first prediction `k=1` are excluded from the
branch search, preserving the recent driving baseline. All retained intervals
count, including narrow fragments. Candidate counts need not increase relative
to the preceding section. This is first-multiple-candidate detection with a fixed
k2 start, rather than the MATLAB planner's k1 start.

The temporary branch-width threshold, count-increase switch, first-transition
switch, and distance-continuity override have been removed from the detector,
node parameters and current configurations. There is one detection rule for both
the optimization's turn-cost window and branch diagnostics.

The turn window spans `max(1, branch_step-turn_preview_steps)..branch_step`.
For `turn_preview_steps=2`, a branch at k2 activates k1..k2; a branch at k5
activates k3..k5. Excluding k1 as a branch anchor does not exclude it from a
later branch's turn-cost window. A horizon of only k1 has no branch anchor.
The detector runs each planning cycle and does not track
junction identity. Removing these heuristics does not merge observed intervals
or delete small spans from the optimizer.

## Diagnostics

`/debug/upper_branch_event` uses schema version 2:

- `candidate_counts`: retained interval counts for k0..N.
- `branch_step`: first section at k>=2 with at least two candidates, or -1.
- `turn_branch_step`: actual solve's turn-cost anchor, using the same rule.

Filtered-count fields and the removed switch/threshold fields are no longer
emitted. `/debug/upper_miqp_trace` field 14 remains the actual turn-cost anchor.
Saved historical events and session configurations retain their original data;
these files are not rewritten by the cleanup.

Regression cases include the recorded 0.1 m fragment at k5, constant two-arm
sections beginning at k1, decreasing candidate counts, and the exclusion of k0/k1.
A planner-level test checks that two candidates at every future section activate
the actual turn-cost anchor at k2 even when the second interval is narrow,
while retaining k1 inside the k2 branch's guidance window.

## Actual optimizer candidate exclusion

`road_candidate_min_width_m` is a separate parameter (default 0, active baseline 0).
Zero disables actual optimizer candidate exclusion as well; every observed
candidate remains selectable, including spans narrower than 2m.
It uses original observed width before contraction, with the same1e-9m threshold
comparison tolerance. At every future stage k1..N, narrower candidates cannot be
selected: direct QP enumeration omits them and the MIQP selector is fixed to0.
If no candidate survives at a stage, solving fails; excluded candidates are never
restored. B measurements, contraction, waypoints, position targets, and observed
geometry stay unchanged. Original candidate indices remain stable in diagnostics.

Events include the applied `road_candidate_min_width_m`,
`optimization_candidate_counts`, and `optimization_candidate_eligible` masks.
The existing `candidate_counts` and interval geometry describe observations,
including excluded candidates. A selected candidate must have a true mask entry.
k0 is not an optimization decision, so its optimization count is0.
