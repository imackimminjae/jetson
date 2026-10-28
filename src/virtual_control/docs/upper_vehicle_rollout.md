# Physical upper reference rollout (experimental)

`upper_use_vehicle_rollout` addresses the abrupt first straight segment of the
upper reference. It starts a continuous curve at the current tracked-point
position, body yaw, and lower-controller effective-steering estimate. The lower
reference receives the integrated curve, rather than straight lines between
0.5-second prediction points. This is opt-in and the canonical configuration
leaves it disabled. It adds no heading-maintenance or path-smoothing cost.

## Model and optimization

The existing QP/MIQP first selects a corridor sequence and provides an initial
path. A bounded nonlinear shooting refinement then holds that sequence, its B
bounds, and the original position/input/turn cost weights fixed. It optimizes
speed and steering-command increments. This local refinement does not enumerate
alternative corridor sequences and does not establish global feasibility.

The state is `[x, y, body_yaw, effective_steering, speed]`. Positions refer to the
same point `offset` metres ahead of the rear axle used by the lower controller.
With wheelbase `L`, effective steering `delta`, command `u`, and actuator time
constant `tau`, the model is:

```
beta      = atan(offset / L * tan(delta))
x_dot     = v * cos(body_yaw + beta)
y_dot     = v * sin(body_yaw + beta)
yaw_dot   = v * cos(beta) * tan(delta) / L
delta_dot = (u - delta) / tau
v_dot     = dv[k] / prediction_dt
```

Commands ramp continuously from the latest lower command to each optimized knot.
RK4 integrates the state every 0.01 seconds by default. Thus the initial curve
direction is `body_yaw + beta`, not necessarily body yaw. The finite first chord
differs slightly from the exact initial tangent because it covers 0.01 seconds.
The initial speed retains the upper planner's nominal-speed convention (6 m/s
in these trials); it is not initialized from measured speed. Speed increments
remain optimized with the original acceleration bound.

Original input penalties are evaluated on speed increments and the physical
coarse-course heading corrections; original relative-turn targets are retained.
Because the motion model changes, their numeric costs can differ from the
linear seed even though the weights and target definitions are unchanged.

The existing `upper_max_steering_rate_radps` now bounds the actual command ramp
and modeled effective-steering derivative. The older path-equivalent rate rows
are bypassed in this mode. Commands and effective steering must stay within the
existing maximum angle. If the current command/estimate already violates the
requested derivative limit, the solve is rejected.

The SQP uses continuous DAQP subproblems, finite differences, trust limits, and
backtracking. Internal restoration slacks help find feasible iterates; an
accepted path still has to satisfy the original selected coarse corridor bounds
within 0.001 m. Every fine point in the published prefix must also lie in a
known drivable cell of the current BEV. This is a center-point check, not a full
swept-vehicle-footprint proof. Existing width and driving guards remain active.
No unconstrained fallback is published. On rejection the existing last-path
retention and lower reference timeout policies apply.

## Parameters and dependencies

All parameters belong to `upper_planner_node`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `upper_use_vehicle_rollout` | `false` | Enable physical refinement and fine curve publication. |
| `upper_vehicle_integration_dt_sec` | `0.01` | Maximum RK4 step; each prediction interval is divided evenly. |
| `upper_vehicle_rollout_max_iterations` | `12` | Local refinement iteration cap. |
| `upper_vehicle_rollout_budget_ms` | `60.0` | Refinement wall-clock budget, checked between operations; not a hard real-time deadline and excludes the initial QP/MIQP. |
| `upper_max_steering_rate_radps` | `0.0` | Must be positive in this mode; trials use `0.5`. |
| `preview_interval_k1_disable_extra_centering` | `false` | At k1 only, cap the regular inset at the boundary-only inset. |

The optional k1 setting applies `inset = min(regular_inset,
min(boundary_margin, 0.45 * raw_length))` after the original interval treatment.
It only expands or preserves the first admissible strip. In particular it does
not tighten a narrow interval whose original inset is already below the boundary
margin. B selection/update, candidate topology, k0 and k2..kN, motion constraints,
costs and fine-reference BEV acceptance are unchanged. This is separate from the
proposed B-source distance gate, which has not been implemented. The interval
info `on` flag still reports the original B/length classification; raw/processed
widths report the actual result after the optional k1 cap. Branch events append
`k1_extra_centering_disabled` to identify the enabled profile.

The cumulative-increment model and fresh lower feedback are required. Existing
`upper_steering_state_topic` and `upper_steering_state_timeout_sec` apply. The
lower trace supplies timestamp 0, effective estimate 19, command 20, actuator
enabled flag 21, delay 22, time constant 23, and reference-point offset 30.
The estimate is an actuator-model state, not measured wheel angle. Source and
receipt ages are checked; the source timestamp must remain fresh after solving.
Only zero actuator delay is currently supported, because delayed-command history
is unavailable to the upper planner. Missing/incompatible feedback rejects a
new path instead of assuming zero steering.

`/debug/upper_branch_event` appends `vehicle_rollout_enabled`,
`vehicle_rollout_valid`, `vehicle_rollout_status`, `vehicle_rollout_ms`,
`vehicle_initial_course_error_rad`, `vehicle_max_command_rate_radps`,
`vehicle_max_effective_rate_radps`, and `vehicle_coarse_states`. Each coarse-state
row is `[time, world_x, world_y, world_body_yaw, effective_delta, speed, command]`.
Numeric upper/lower trace layouts are unchanged. Sparse paths remain coarse
diagnostics; lower-reference topics contain the fine integrated curve and its
arc lengths.

## Validation and limits

Artifacts and separate S1/S3 trial configurations are in
`followup_20261016/upper_vehicle_rollout/`. Pure tests compare straight motion,
offset-point circles, and actuator ramps to analytic solutions. Integration
tests exercise physical refinement, strict infeasible-road rejection, and
actual fine-reference publication on an isolated ROS domain without a lower
controller or hardware outputs.

Frozen replay holds recorded pose, prior steering, reference, and corridor/B
choices fixed. Counterfactual paths do not affect subsequent frames, so it
cannot establish closed-loop stability or driving quality. Recorded BEV maps
are associated using raw interval endpoint matching because map header stamps
are zero; acquisition-pose ambiguity remains. See `summary.json` and
`RESULT.txt` for measured results and unresolved rejection rates. The feature
stays disabled in the canonical configuration pending further validation.
