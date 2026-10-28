# Upper steering-command change cost

`upper_r_steering_delta` adds a direct command-change penalty to the physical
upper rollout. It is finite and nonnegative; default `0.0` preserves the previous
objective. A positive value requires `upper_use_vehicle_rollout=true`.

The current scenario-bias simulation profile enables `3.0`. A frozen replay of
351 recorded S1 frames compared 0/3/10/30 with the same production solver and
preferred/raw geometry. All four settings produced 351 valid plans. At 3.0,
mean within-horizon command total variation fell 86.1%, while the largest
preferred-strip violation rose from 0.482 m to 0.515 m. These are independent
frozen-frame planning results, not new driving or a measured reduction in
vehicle oscillation. Inputs, metrics and rollback copies are under
`diagnostics/upper_steering_change_20261025/`.

For optimized command knots `delta[k]`, it adds

    J_change = 0.5 * upper_r_steering_delta * sum((delta[k] - delta[k-1])^2).

Angles are radians. The first difference starts from the actual latest lower
command supplied in steering feedback, rather than zero. Later differences
compare adjacent optimized command knots. Existing continuous interpolation
between knots and actuator dynamics remain unchanged. At the current 0.5 s upper
prediction spacing, this penalizes command increments per 0.5 s; it is not
normalized by angle, horizon length or prediction period.

The term participates in SQP residuals, gradients, Hessians, line search and
best-feasible-solution selection. It does not alter the initial MIQP corridor
selection or impose agreement with the previous planned path. In particular,
this is independent of the existing reference-relative heading-increment cost
`upper_r_dpsi`, branch turn guidance, and lower MPC steering-change cost.

The numeric trace layout is preserved: `input_cost` includes this penalty, so
the objective decomposition still sums correctly. Branch-event JSON additionally
reports `steering_change_weight` and `steering_change_cost` separately; do not
add that cost a second time when summing the objective.

Road bounds, steering angle/rate bounds, B and preferred-strip geometry, branch
detection and ordinary-refresh reset policy are unchanged. A higher weight can
trade tracking or preferred-strip accuracy for smaller command changes. Frozen
input replay can verify this tradeoff but cannot establish that a subsequent
closed-loop drive will be smoother.
