# Upper-planner steering-rate constraint

This document describes `upper_use_vehicle_rollout=false`. The optional physical
mode replaces the equivalent-steering proxy with integrated vehicle/actuator
dynamics and a fine curved lower reference; see [upper_vehicle_rollout.md](upper_vehicle_rollout.md).

The optional constraint connects the current lower-controller effective-steering
estimate to the first upper prediction interval and limits changes between later
intervals. It adds hard inequalities and independent output checks. It does not
add a heading cost or change interval/B extraction, road boundaries, lower
controller settings, planning frequency, or prediction spacing.

## Parameters

All parameters belong to `upper_planner_node`.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `upper_max_steering_rate_radps` | `0.0` | Zero disables. Positive limits the magnitude of the model-equivalent steering rate in rad/s. |
| `upper_steering_state_topic` | `/debug/lower_mpc_trace` | Existing lower trace; field 19 is the effective-steering estimate in radians, field 0 is its ROS timestamp. |
| `upper_steering_state_timeout_sec` | `0.25` | Maximum source timestamp and reception age. Must be positive. |

The feature requires `upper_cumulative_increment_model=true`. Missing, stale,
future-dated, malformed or nonfinite feedback prevents a new plan; it never
silently assumes zero steering or disables the new constraint. The previous-path
retention and lower path timeout policies remain in force. Lower neutral/waiting
traces also supply estimates, so initialization does not require a successful
upper path first. Topic field 19 is an internal actuator-model estimate, **not a
measurement of wheel angle**. Feedback and planner must use the same ROS clock.
The source estimate must remain fresh at completion of the solve as well.

## Formulation

For total upper-model inputs `dv[k], dpsi[k]`, including the reference increments:

```
v[k]     = nominal_speed + sum(dv[0:k])
delta[k] = atan(wheelbase * dpsi[k] / (prediction_dt * v[k]))
h        = max_steering_rate * prediction_dt
```

The first predicted equivalent angle is bounded by `initial_estimate +/- h` and
the original maximum angle. Those first-step inequalities are exact and linear
in speed and heading increment. The anchor is not clipped to the upper angle
limit: if it cannot reach that limit in one interval the problem is rejected.

For later intervals we retain the existing QP/MIQP solver using sufficient,
conservative linear inequalities:

```
L/dt * abs(dpsi[k] - dpsi[k-1]) + tan(max_angle) * abs(dv[k])
    <= v_min[k] * h
```

Four sign combinations implement these two absolute values. The lower speed
bound is `max(min(0.1, nominal_speed), nominal_speed-(k+1)*max_dv)`. A matching
positive-speed inequality is included. The inequalities, together with the
original acceleration and maximum-angle constraints, guarantee the rate bound
for the equivalent model steering: `atan` is 1-Lipschitz and the previous angle
is bounded. This also covers acceleration/deceleration; simply limiting a
fixed multiple of `dpsi[k]-dpsi[k-1]` would not.

These are sufficient conditions, not an exact nonlinear steering-rate feasible
set. They become more conservative at the far end of a horizon with a low
possible speed. Speed is still optimized; no speed or road constraint is relaxed.
The same rows participate in direct corridor QPs, full MIQP and polishing.

The constructed linear rows use 98% of the requested rate to leave small
headroom for geometric conversion. This is a conservative construction margin,
not acceptance slack. Before a new path is accepted, two checks verify it:

1. Exact `atan` conversion of the total model inputs with their optimized speeds
   checks the existing maximum angle and the requested rate.
2. The requested rate of the resulting geometric polyline is independently checked.
   Equivalent steering is calculated as follows: each segment's
   wrapped heading change from the previous heading (measured body heading for
   the first) is divided by its length and multiplied by wheelbase, then `atan`.

The existing maximum-angle bound remains a model constraint; this feature does
not add a different maximum-curvature bound on the geometric polyline. The
steering-angle difference tolerance is `1e-6 rad`; the rate tolerance is this divided
by prediction dt. A geometric rate violation or model violation rejects the result rather than
publishing a violating path. A constrained QP solution can therefore be rejected
by this geometric check; no unconstrained fallback is substituted. It is a
finite-interval equivalent-steering check, not an instantaneous tangent,
continuous swept-footprint or full actuator-lag feasibility guarantee. The upper
model still has no 0.74-second actuator state. Closed-loop validation is required.

## Diagnostics and trials

`/debug/upper_branch_event` keeps existing fields and appends
`steering_rate_limit_radps`, `steering_anchor_rad`, `steering_anchor_stamp`,
`steering_anchor_age_sec`, `model_steering_rad`, `geometric_steering_rad`,
`max_model_steering_rate_radps`, and `max_geometric_steering_rate_radps`.
Existing numeric upper/lower trace layouts are unchanged. The recorder already
records the lower trace and branch event.

The canonical configuration leaves the new limit at zero. Separate S1/S3 trial
configurations, frozen closed-log replay inputs/results and figures are under
`followup_20261016/upper_steering_rate/`. Frozen replay holds recorded pose,
reference, processed road corridors, waypoint pair and preceding steering
estimate fixed. Counterfactual paths do not feed the next cycle or a simulated
vehicle; success/heading changes from replay do not establish driving quality.
