# Minimum width for branch turn-cost detection

`branch_min_candidate_width_m` defaults to **2.0 m**. The active
`scenario_bias_baseline.yaml` sets it to **0.0 m** (disabled).
It is finite and nonnegative; zero disables
width filtering. A run is eligible when its **original observed cross-section
length**, before center contraction or boundary margins, is at least this value.
A 1e-9 m comparison tolerance accepts an exact threshold despite floating-point
rounding. This is a lateral scan length, not a calibrated physical lane width.

Let C[k] be the number of width-eligible candidates. The detector searches
k=2..N and uses the first step satisfying C[k-1] >= 1 and C[k] > C[k-1].
Thus it detects increases such as 1->2 and 2->3, and excludes the k0->k1
transition entirely. Unchanged or decreasing counts do not activate costs.
A missing eligible previous section (zero candidates) is not a split.
Candidate geometry and the selected corridor do not change this decision;
the distance-based continuity heuristic has been removed.

The turn-cost window spans max(1,k-turn_preview_steps) through k. With
turn_preview_steps=2, a count increase at k2 activates k1..k2, and an increase
at k5 activates k3..k5. Excluding k0->k1 from detection does not prohibit k1
from belonging to a later split's cost window. Constant two-arm sections
throughout k1..N have no turn window. The detector runs again each planner
cycle; it does not track junction identity or passage.

Width filtering does not delete drivable pixels, merge blocked gaps, change B
sampling or contraction, or remove road constraints/corridor choices. A preferred
width below 2 m remains branch-eligible when its original width is at least 2 m.

`/debug/upper_branch_event` provides:

- `branch_min_candidate_width_m`: applied threshold.
- `branch_candidate_counts`: eligible counts at k0..N; `candidate_counts` keeps
  all retained road candidates for compatibility.
- `filtered_branch_step`: first eligible count increase at k>=2, or -1.
- `turn_branch_step`: actual solve's turn-window anchor, or -1.

The existing event `branch_step` still denotes the first raw multiple-candidate
step. `/debug/upper_miqp_trace` field 14 remains the actual turn-window anchor,
and the appended window-step/target/weight triples show whether costs are active.
The former connected-split diagnostic is no longer emitted.

Recorded S1 fixtures cover the 18.326 s boundary sliver (0.1 m; filtered out)
and the 21.426 s frame (two eligible candidates throughout k1..N; no turn
window). Unit tests also cover exact-threshold inclusion, original width
eligibility, k0->k1 exclusion, later 1->2 and 2->3 transitions, unchanged and
decreasing counts, and a missing eligible previous section.

Validation uses unit tests and frozen recorded widths; it does not establish
new closed-loop steering performance. A new drive is required to measure that.

Independent rollback comparisons can override `branch_require_count_increase`
(default true), `branch_include_k0_transition` (default false), and
`branch_use_split_continuity` (default false). The latter enables the historical
distance-based anchor override with `branch_split_continuity_m` (default 6.75).
Each override preserves the other selected width and k0 eligibility settings;
enabling continuity alone is therefore not a full historical-profile restore.
The active baseline disables the count-increase requirement and both width
thresholds, excludes the k0 transition, and disables the continuity override. Recorded branch events include
all four switches/thresholds so a comparison can verify their applied values.


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
