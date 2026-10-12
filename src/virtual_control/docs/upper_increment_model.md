# Upper planner increment consistency

**Status: rolled back on 2026-09-24 at 04:31 KST after the next driving log
confirmed an initial counter-turn and abrupt replan. The description below is
the tested candidate, not the active configuration.** See
`drive_debug_20260924_042235_light/analysis/REPORT.md`.

The split profile enables `upper_consistent_increment_model: true`.
The default is false for compatibility with other profiles; false selects the
original model for controlled A/B comparisons. Restart the upper node after
changing this parameter. It is read at startup, not dynamically reconfigured.

Previously the position Jacobian treated each dv/dpsi correction as a change
in one segment only, while vehicle constraints and turn cost accumulated those
same variables as increments. In a branch this allowed predicted heading and
published path direction to disagree substantially.

With the switch enabled:

- Each speed/heading increment affects every subsequent displacement.
- The heading Jacobian uses the actual reference segment length. Nominal speed
  increments use segment length / dt, so resampling does not introduce another
  mismatch between reference geometry and speed constraints.
- The branch target remains a fraction of the road turn. Its incoming-road
  frame is converted to a correction to the reference heading before adding
  the cost. The trace's `psi_pred_cost` is now predicted heading relative to the
  incoming road; `psi_pred_model` stays in the measured body frame.

This remains a first-order position model. The angle of a resulting segment
is not exactly the nonlinear heading state for large corrections. The change
does not add initial-heading continuity constraints, curvature feedforward,
soft-ratio changes, or lower-controller tuning.

## Rollback

The pre-change sources, split profile, test source and upper executable are
saved with SHA256 checksums in:

`/home/imac/ros2_ws/rollback/upper_increment_model_20260924_040637`

Restore without rebuilding:

```bash
python3 /home/imac/ros2_ws/rollback/upper_increment_model_20260924_040637/rollback.py
```

The script stops this workspace's upper/lower controllers, archives the state
being replaced, then restores the exact originals. It leaves controllers
stopped; restart them normally. MAVProxy and bridges are not stopped. A
controller that fails to stop aborts restoration. To check backup integrity
without changing anything, add `--check`.

Restoration and preservation of replaced files were tested in a temporary
workspace; actual vehicle controllers were not launched for this experiment.

## Validation (2026-09-24)

- Package build passed (existing Eigen/NEON warnings).
- 42 test cases across four CTest targets passed: path generation
  (6), interval state (13), geometry (19), and MIQP constraints (4).
- New cases check repeated left/right increments against a nonlinear rollout
  and verify that an already aligned curved reference gets no extra turn.
- Bag: `drive_debug_20260924_035423_light/bag`, same input messages, independent
  localhost ROS domains, MAVLink/PWM disabled. Both original and corrected
  models produced 121 valid plans, all with horizon 5. Each replay had 601
  valid lower QPs and 25 startup samples without a path.
- In the 03:55:01–03:55:18 branch window, steering total variation changed
  304.63 -> 268.75 degrees (-11.8%), but saturation increased
  14.7% -> 37.6% and RMS increased 10.88 -> 12.93 degrees.
- Over branch plans, maximum segment-heading/model-heading mismatch changed
  38.74 -> 4.15 degrees (median 11.67 -> 0.34 degrees).

The corrected model resolves the representation inconsistency. It has **not**
yet demonstrated improved closed-loop vehicle behavior. Recorded poses follow
the old commands, so a new path produces counterfactual tracking errors in this
replay. More saturation is an unresolved concern, not evidence to omit.

Detailed replay metrics and plots are in
`drive_debug_20260924_035423_light/analysis/increment_comparison.json` and
`increment_comparison.png`.

### Diagnostic turn-cost ablation

The same corrected model was also replayed with turn weights 0.3 and 0.0
(command-line overrides only). Both produced 121 valid five-step plans.

| Turn weight | Steering variation (deg) | First-heading variation (deg) | Saturation |
|---|---:|---:|---:|
| 0.9 (active profile) | 268.75 | 243.53 | 37.6% |
| 0.3 (diagnostic) | 271.20 | 225.80 | 23.5% |
| 0.0 (diagnostic) | 183.35 | 121.63 | 21.8% |

Turning the cost off reduces oscillation in replay, but does not remove it.
The profile retains weight 0.9 so the next driving experiment isolates the
model/frame correction. Lowering the weight is not established as a complete
fix by this fixed-pose replay. The active change is an experiment ready for
the user's next normal launch, with exact pre-change rollback available.
