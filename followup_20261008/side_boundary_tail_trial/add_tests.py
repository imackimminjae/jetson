from pathlib import Path
H=Path(__file__).resolve().parent;p=H/'trial/src/virtual_control/test/test_lower_path_generation.cpp';s=p.read_text();needle='  static auto densify(SdMapUpperPlannerNode & node, std::vector<double> & s)'
s=s.replace(needle,'''  static auto sideLoss(SdMapUpperPlannerNode & node, int variant, int side, double angle)
  {
    SdMapUpperPlannerNode::GridMapSnapshot grid;
    grid.valid = true; grid.width = grid.height = 128; grid.resolution = 0.3125;
    grid.origin_yaw_rad = angle;
    const double c = std::cos(angle), sn = std::sin(angle);
    Eigen::Matrix2d rotation; rotation << c, -sn, sn, c;
    grid.origin_body = rotation * Eigen::Vector2d(0, -20);
    grid.data.assign(128 * 128, 100);
    for (int y = 0; y < 128; ++y) {
      for (int x = 0; x < 128; ++x) {
        const double forward = (x + 0.5) * grid.resolution;
        const double lateral = side * (-20 + (y + 0.5) * grid.resolution);
        const bool near = forward > 14 && forward < 18;
        const bool far = variant == 1 && forward > 18;
        if (std::abs(lateral) < 3 || ((near || far) && lateral > 15)) {
          grid.data[y * 128 + x] = 0;
        }
        if (variant == 2 && near && lateral > 18.5) {
          grid.data[y * 128 + x] = -1;  // Interior unknown is not the map edge.
        }
      }
    }
    SdMapUpperPlannerNode::PreviewReferenceData ref;
    ref.valid = true; ref.used_previous_solution = variant != 3;
    for (const auto & point : std::vector<Eigen::Vector2d>{{0,0},{8,0},{16,18.0*side},{20,21.0*side}}) {
      ref.points_body.push_back(rotation * point);
      ref.heading_rad.push_back(angle);
    }
    return node.extractPreviewIntervals(ref, &grid, {});
  }
  static auto prefixOnly(const SdMapUpperPlannerNode::PreviewConstraintData & p)
  {
    SdMapUpperPlannerNode::UpperSolveResult r;
    r.valid = true; r.predicted_body = p.prk;
    auto published = SdMapUpperPlannerNode::publishablePrediction(p, r);
    return std::make_pair(published, r.predicted_body);
  }

'''+needle)
needle='TEST(LowerPathGeneration, K1TrialOnlyMarksObservedFrontClippedCandidates)'
s=s.replace(needle,'''TEST(LowerPathGeneration, SideLossRequiresReferenceConnectedMapEdgeAndMissingTail)
{
  rclcpp::init(0, nullptr);
  for (bool enabled : {false, true}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("guard_side_boundary_terminal_loss", enabled),
      rclcpp::Parameter("grid_positive_is_drivable", false),
      rclcpp::Parameter("grid_value_threshold", 0)});
    auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
    using Access = imac_ctrl::LowerPathGenerationTestAccess;
    Access::cancel(*node);
    for (int side : {-1, 1}) {
      for (double angle : {0.0, 0.7}) {
        for (int variant : {0, 1, 2, 3}) {
          const auto p = Access::sideLoss(*node, variant, side, angle);
          ASSERT_EQ(p.N_pred, 3);
          const bool expected = enabled && variant == 0;
          EXPECT_EQ(p.side_boundary_tail_start, expected ? 3 : -1);
          EXPECT_EQ(p.side_boundary_side, expected ? side : 0);
          EXPECT_EQ(p.prk.size(), 4u);  // Reference/model size never changes.
          const auto result = Access::prefixOnly(p);
          EXPECT_EQ(result.first.size(), expected ? 3u : 4u);
          EXPECT_EQ(result.second.size(), 4u);  // Full solution still available for carry.
          for (std::size_t j = 0; j < result.first.size(); ++j) {
            EXPECT_LT((result.first[j] - p.prk[j]).norm(), 1e-12);
          }
        }
      }
    }
  }
  rclcpp::shutdown();
}

'''+needle);p.write_text(s)
