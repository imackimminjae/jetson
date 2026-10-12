from pathlib import Path
H=Path(__file__).resolve().parent;T=H/'trial/src/virtual_control'
p=T/'include/virtual_control/tracking_control.hpp';s=p.read_text()
s=s.replace('    int N_pred{0};','''    int N_pred{0};
    // Full model/reference remain intact. A lost, side-clipped terminal road
    // is not a forced alternative; only the constrained prefix is published.
    int side_boundary_tail_start{-1};
    int side_boundary_side{0};''')
s=s.replace('  void applyTerminalPositionTargets(','''  static std::vector<Eigen::Vector2d> publishablePrediction(
    const PreviewConstraintData & preview, const UpperSolveResult & solve);
  void applyTerminalPositionTargets(''')
s=s.replace('  bool skip_front_clipped_k1_lane_constraints_{false};','''  bool skip_front_clipped_k1_lane_constraints_{false};
  bool guard_side_boundary_terminal_loss_{false};''');p.write_text(s)
p=T/'src/tracking_control.cpp';s=p.read_text()
s=s.replace('  declare_parameter<bool>("preview_restore_full_horizon", preview_restore_full_horizon_);','''  declare_parameter<bool>("preview_restore_full_horizon", preview_restore_full_horizon_);
  declare_parameter<bool>("guard_side_boundary_terminal_loss", guard_side_boundary_terminal_loss_);''')
s=s.replace('  get_parameter("preview_restore_full_horizon", preview_restore_full_horizon_);','''  get_parameter("preview_restore_full_horizon", preview_restore_full_horizon_);
  get_parameter("guard_side_boundary_terminal_loss", guard_side_boundary_terminal_loss_);''')
needle='  observations.resize(static_cast<std::size_t>(output.N_pred + 1));'
assert s.count(needle)==1
s=s.replace(needle,'''  // Detect loss of the reference-connected road at a lateral map boundary.
  // Work on raw intervals, before B/soft contraction. Do not infer clipping
  // from candidate count alone, blocked cells, or an unknown interior gap.
  const int last = output.N_pred;
  if (guard_side_boundary_terminal_loss_ && reference.used_previous_solution &&
    grid && grid->valid && last == requested_horizon && last >= 2 &&
    output.Nck[last] > 0)
  {
    const double c = std::cos(grid->origin_yaw_rad);
    const double s = std::sin(grid->origin_yaw_rad);
    const auto local = [&](const Eigen::Vector2d & p) -> Eigen::Vector2d {
        const Eigen::Vector2d q = p - grid->origin_body;
        return {c * q.x() + s * q.y(), -s * q.x() + c * q.y()};
      };
    const double height = grid->height * grid->resolution;
    const double tolerance = preview_line_sample_m_ + 1e-6;
    const auto clipped = [&](int k, std::size_t j, int side) {
        const auto & obs = observations[k][j];
        if (obs.nominal_fallback) {return false;}
        const double edge = side < 0 ? 0.0 : height;
        return (!obs.start_boundary_observed &&
               std::abs(local(output.pmk[k][j]).y() - edge) <= tolerance) ||
               (!obs.end_boundary_observed &&
               std::abs(local(output.pMk[k][j]).y() - edge) <= tolerance);
      };
    bool reference_present = false;
    for (const auto & obs : observations[last]) {
      reference_present = reference_present || obs.reference_inside || obs.nominal_fallback;
    }
    const Eigen::Vector2d tail = local(output.prk[last]);
    const Eigen::Vector2d before = local(output.prk[last - 1]);
    const double step = (tail - before).norm();
    for (const int side : {-1, 1}) {
      const double clearance = side < 0 ? tail.y() : height - tail.y();
      if (reference_present || clearance > step + grid->resolution ||
        side * (tail.y() - before.y()) <= 0.0)
      {
        continue;
      }
      bool connected_clipped = false;
      for (std::size_t j = 0; j < observations[last - 1].size(); ++j) {
        connected_clipped = connected_clipped ||
          (observations[last - 1][j].reference_inside && clipped(last - 1, j, side));
      }
      bool boundary_candidate_remains = false;
      for (std::size_t j = 0; j < observations[last].size(); ++j) {
        boundary_candidate_remains = boundary_candidate_remains || clipped(last, j, side);
      }
      if (connected_clipped && !boundary_candidate_remains) {
        output.side_boundary_tail_start = last;
        output.side_boundary_side = side;
        break;
      }
    }
  }

'''+needle)
needle='void SdMapUpperPlannerNode::applyTerminalPositionTargets('
s=s.replace(needle,'''std::vector<Eigen::Vector2d> SdMapUpperPlannerNode::publishablePrediction(
  const PreviewConstraintData & preview, const UpperSolveResult & solve)
{
  if (!solve.valid) {return {};}
  auto points = solve.predicted_body;
  if (preview.side_boundary_tail_start >= 2 &&
    static_cast<std::size_t>(preview.side_boundary_tail_start) < points.size())
  {
    points.resize(static_cast<std::size_t>(preview.side_boundary_tail_start));
  }
  return points;
}

'''+needle)
s=s.replace('  const auto skip_lane = [&](int k, int candidate) {','''  const auto skip_lane = [&](int k, int candidate) {
      if (guard_side_boundary_terminal_loss_ && preview.side_boundary_tail_start >= 2 &&
        k >= preview.side_boundary_tail_start) {return true;}''')
# Preserve full solved plan for previous-reference memory; only send known prefix.
s=s.replace('  const auto dense_world = densifyPath(sparse_world, planning_speed_mps, dense_s);','''  std::vector<Eigen::Vector2d> published_world;
  for (const auto & point : publishablePrediction(constraints, solve)) {
    published_world.push_back(pose.position + rotation_world_body * point);
  }
  const auto dense_world = densifyPath(published_world, planning_speed_mps, dense_s);''')
s=s.replace('  publishPath(sparse_path_pub_, sparse_world, frame_id);','  publishPath(sparse_path_pub_, published_world, frame_id);')
needle='  out << ",\\\"retained_world\\\":";'
assert s.count(needle)==1
s=s.replace(needle,'''  out << ",\\\"side_boundary_tail_start\\\":" <<
    (preview ? preview->side_boundary_tail_start : -1);
  out << ",\\\"side_boundary_side\\\":" << (preview ? preview->side_boundary_side : 0);
  out << ",\\\"published_world\\\":";
  points(preview && solve ? publishablePrediction(*preview, *solve) :
    std::vector<Eigen::Vector2d>{}, true);
'''+needle)
s=s.replace('    for (const auto & p : solve->predicted_body) {diagnostic_last_path_.push_back(world(p));}','''    for (const auto & p : publishablePrediction(*preview, *solve)) {
      diagnostic_last_path_.push_back(world(p));
    }''')
p.write_text(s)
p=T/'config/tracking_control_split.yaml';s=p.read_text();s=s.replace('    preview_restore_full_horizon: true','''    preview_restore_full_horizon: true
    # Trial: omit only a terminal road lost at a lateral BEV edge. Keep the
    # full reference/model; publish only the road-constrained prefix.
    guard_side_boundary_terminal_loss: false''');p.write_text(s)
