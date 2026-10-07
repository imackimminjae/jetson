Grid-origin trial, 2026-10-03
User requested treating the current image start as vehicle x=0.
Active split config: grid_origin_x_override_enabled=true, grid_origin_x_override_m=0.0.
The planner changes only its internal grid origin x; all cells, resolution, y,
orientation and source ROS messages are preserved. A received origin 2.95m
therefore produces an effective 0..40m grid. This is a coordinate hypothesis
experiment, not generation of previously unseen near-field data.
The source /bev/occupancy_grid remains at its published origin, so RViz/raw bags
continue to show the source origin. Planner diagnostics print received/effective x.
Disable grid_origin_x_override_enabled and restart upper_planner_node to restore
normal interpretation. No vehicle motion or ARM was initiated.

The preceding clipped-interval attempt was saved to
rollback/20261003_clipped_interval_attempt_saved and removed from active sources.
Its C++ tests passed but its legacy ROS integration suite did not complete;
that attempt should not be treated as a fully validated finished change.
This experiment uses the pre-attempt interval behavior to isolate the origin change.

Validation: colcon build completed; isolated grid-origin ON/OFF coordinate test passed; geometry, interval-state and MIQP-constraint CTest suites passed. Production nodes were not started or restarted. Restart the upper planner to load the experiment settings.
