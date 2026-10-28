# Hard observed road, soft preferred intervals

`upper_soft_corridor_constraints=true` separates the observed drivable run from
its preferred subinterval. This changes the meaning of the existing margin and
centering strip: outside that strip is allowed at a finite cost, while the
uncontracted observed road remains a hard constraint. It does not preserve a
mandatory 2 m clearance. No previous-plan heading cost is added.

## Bounds and objective

The existing B update, soft-ratio contraction and boundary-margin calculation
produce the preferred endpoints. The enabled profile disables both the legacy
k1 lane-skipping option and the k1 extra-centering cap, so k1 uses the same
preferred-strip treatment as later stages. The
uncontracted endpoints are retained separately, with the same candidate indices.
For each selected road candidate and prediction station, with projection `z`:

- Hard: `road_lower <= z <= road_upper`, subject to the existing numerical
  feasibility tolerance (default 0.001 m).
- Soft: `preferred_lower - slack_lower <= z <= preferred_upper + slack_upper`,
  with nonnegative continuous slack in metres.
- Additional cost: `linear_weight * (slack_lower + slack_upper)` plus
  `0.5 * quadratic_weight * (slack_lower^2 + slack_upper^2)`.

The cost is optimized together with the existing position, turn and input costs;
it is not a lexicographic constraint. Both the direct fixed-combination QPs and
the mixed-integer fallback include this cost. Physical refinement uses the same
hard road and soft preferred intervals, including its acceptance check. Its
internal hard-feasibility restoration slack is still rejected on acceptance.
Soft violations do not independently reject an otherwise feasible physical path.

`upper_corridor_soft_linear_weight` defaults to 1.0 (finite, nonnegative).
`upper_corridor_soft_quadratic_weight` defaults to 10.0 (finite, positive).
These are initial, uncalibrated weights, not a validated driving recommendation.
The existing `soft_ratio` is a geometric ratio; it is not a penalty weight.

## Observation and publication checks

A soft-mode solve requires explicit raw road candidates. A nominal fallback
interval cannot establish an observed hard road. A preferred interval extending
outside its hard road is rejected as an invalid problem. Disconnected drivable
runs remain separate candidates; their gap is never merged into a drivable run.
The legacy k1 lane-skipping option does not bypass hard-road bounds in soft mode.

After solving, actual hard-road residuals and preferred violations are recomputed
from the accepted path. Before committing or publishing a new reference, sampled
dense points must lie in known drivable BEV cells. Unknown, blocked and outside
cells are rejected. The physical mode also retains its existing fine-rollout
BEV check. Existing terminal-loss handling publishes only its constrained prefix.

These are tracked-point and cross-section constraints. They do not constitute a
continuous swept-vehicle-footprint guarantee, calibrated clearance, or acquisition
synchronization proof. The existing response of retaining the last path when a
new solve fails is unchanged; the new road checks do not revalidate a retained
old path against every subsequent BEV. Such a fallback needs separate validation.

## Configuration and diagnostics

The option defaults to false, preserving existing profiles. The implementation
work includes a separate `soft_road_p10_c20_rd600.yaml` control overlay based on
the recorded no-bias profile. It enables soft mode with the two weights above,
without changing the soft ratio, margin setting or vehicle/lower control limits.
It contains upper/lower control parameters only and must be combined with the
appropriate existing bridge, sensor and session configuration.

`/debug/upper_branch_event` keeps the original field offsets and preferred
`intervals_world` representation and adds:

- `road_intervals_world`: original, uncontracted runs `[k,candidate,x0,y0,x1,y1]`.
- `soft_corridor_constraints`, `corridor_soft_linear_weight`,
  `corridor_soft_quadratic_weight`.
- `preferred_corridor_cost`, `max_preferred_corridor_violation_m`,
  `preferred_corridor_violations_m`, `max_hard_road_violation_m`.

Accepted soft-mode paths can have positive preferred violation. Hard-road and
actuator violations remain reasons for rejection. Diagnostics for unavailable
accepted-plan residuals are null, rather than evidence of a zero violation.

## Validation boundary

Unit tests exercise hard rejection, finite preference costs, weight tradeoffs,
separate candidate gaps, continuous slack in the mixed-integer fallback,
physical refinement, and a blocked BEV strip between prediction stations.
Closed-log replay uses frozen pose, feedback, preview, road candidates and cost
targets. Restoring a previously failed input offline is not closed-loop evidence
and does not establish vehicle-footprint safety or controller stability.
No vehicle run is part of this implementation validation.

The saved 353-frame replay results precede the profile correction that disables
the k1 extra-centering cap. At the user's request, no additional verification
was run after that correction; those results do not validate the corrected
profile's preferred geometry.
