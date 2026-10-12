// Offline harness: open-loop replay (recorded pose/grid/reference) and closed-loop simulation
// (production upper + lower code, bicycle plant, recorded BEV frames reprojected to the simulated pose).
// No ROS spinning, no command publication, no hardware I/O.
#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "/home/imac/ros2_ws/src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <random>
extern int exp_b_mode;
extern int exp_recover_cycles;
void exp_reset();
using J = nlohmann::json;
using V = Eigen::Vector2d;
using Points = std::vector<V>;
static Points pts(const J & j) {Points p; for (const auto & q : j) p.emplace_back(q[0].get<double>(), q[1].get<double>()); return p;}
static J packed(const Points & p) {J j = J::array(); for (const auto & q : p) j.push_back({q.x(), q.y()}); return j;}
static Eigen::Matrix2d rotation(double a) {Eigen::Matrix2d r; r << cos(a), -sin(a), sin(a), cos(a); return r;}
static std::vector<int8_t> readBin(const std::string & f) {std::ifstream s(f, std::ios::binary); return std::vector<int8_t>((std::istreambuf_iterator<char>(s)), {});}

struct StaticMap {
  std::vector<int8_t> cells; V origin; int w, h; double res;
  StaticMap(const std::string & bin, const std::string & meta) {
    J m; std::ifstream(meta) >> m; origin = V(m["origin"][0], m["origin"][1]); res = m["res"]; w = m["width"]; h = m["height"]; cells = readBin(bin);
  }
  int cell(const V & q) const {int x = floor((q.x() - origin.x()) / res), y = floor((q.y() - origin.y()) / res); return x >= 0 && y >= 0 && x < w && y < h ? cells[y * w + x] : 100;}
};
struct Frames {
  std::vector<int8_t> data; std::vector<std::array<double, 4>> pose;
  Frames(const std::string & bin, const std::string & poses) {J p; std::ifstream(poses) >> p; for (auto & q : p) pose.push_back({q[0], q[1], q[2], q[3]}); data = readBin(bin);}
  // Nearest recorded frame by position and heading, optionally excluding one source attempt.
  int nearest(const V & xy, double yaw, int exclude) const {
    int best = -1; double c = 1e30;
    for (size_t i = 0; i < pose.size(); ++i) {
      if (static_cast<int>(pose[i][3]) == exclude) continue;
      double dy = std::remainder(yaw - pose[i][2], 2 * M_PI);
      double d = (xy - V(pose[i][0], pose[i][1])).squaredNorm() + 16.0 * dy * dy;
      if (d < c) {c = d; best = static_cast<int>(i);}
    }
    return best;
  }
};
static const V kGridOrigin(0.0, -20.0);

namespace imac_ctrl {
struct LowerTrackingMpcTestAccess {using Impl = LowerTrackingMpcNode::Impl; static Impl & get(LowerTrackingMpcNode & n) {return *n.impl_;}};
struct LowerPathGenerationTestAccess {
  using Node = SdMapUpperPlannerNode;
  struct Memory {Points previous; LowerPathGeometry path; Node::IntervalCenteringState interval; int wp = 0; double last_success = -100;};

  static J intervalsSummary(const Node::PreviewConstraintData & pc) {
    J r = J::array();
    for (size_t k = 0; k < pc.interval_info.size(); ++k)
      for (size_t i = 0; i < pc.interval_info[k].size(); ++i) {
        const auto & a = pc.interval_info[k][i];
        r.push_back({k, i, a.observation.length, a.processed_length, a.treatment.centering_on ? 1 : 0, a.observation.reference_inside ? 1 : 0});
      }
    return r;
  }

  // Open-loop: one attempt's recorded cycles. Reference, waypoint pair and pose are the recorded ones.
  static J openLoop(Node & n, const J & cycles, const std::vector<int8_t> & grids, int attempt) {
    n.planner_timer_->cancel(); exp_reset();
    Points route; J rj; std::ifstream("route.json") >> rj; route = pts(rj["route"]);
    Node::IntervalCenteringState interval; bool first = true; bool prev_valid = false; J out = J::array();
    for (const auto & c : cycles) {
      if (c["attempt"].get<int>() != attempt) continue;
      if (first) {interval.reference_length = c["B_before"].get<double>(); first = false;}
      V position(c["x"].get<double>(), c["y"].get<double>()); double yaw = c["yaw"];
      Node::GridMapSnapshot g; g.valid = true; g.frame_id = "Vehicle"; g.width = g.height = 128; g.resolution = .3125; g.origin_body = kGridOrigin;
      size_t gi = c["grid"].get<size_t>(); g.data.assign(grids.begin() + gi * 16384, grids.begin() + (gi + 1) * 16384);
      auto R = rotation(yaw); auto body = [&](const V & q) -> V {return R.transpose() * (q - position);};
      int wp = c["wp0"], wp1 = c["wp1"];
      Points prefix; for (const auto & q : c["preview"]) prefix.push_back(body(V(q[0].get<double>(), q[1].get<double>())));
      int horizon = static_cast<int>(prefix.size()) - 1;
      auto ref = n.buildPreviewReference(wp > 0 ? body(route[wp - 1]) : V::Zero(), body(route[wp]), body(route[wp1]), wp > 0, n.target_speed_mps_, horizon, prefix);
      ref.used_previous_solution = prev_valid;
      auto pc = n.extractPreviewIntervals(ref, &g, interval); interval = pc.interval_update.state;
      n.applyTerminalPositionTargets(pc, body(route[wp]), body(route[wp1]), n.target_speed_mps_ * n.upper_prediction_dt_sec_);
      auto z = n.solveUpperMiqp(pc, n.target_speed_mps_);
      J row = {{"t", c["t"]}, {"x", position.x()}, {"y", position.y()}, {"yaw", yaw}, {"valid", z.valid}, {"B", interval.reference_length},
        {"B_candidate", pc.interval_update.candidate_length}, {"B_candidate_step", pc.interval_update.candidate_step}, {"intervals", intervalsSummary(pc)}};
      row["published"] = J::array(); for (const auto & q : Node::publishablePrediction(pc, z)) {V p = position + R * q; row["published"].push_back({p.x(), p.y()});}
      prev_valid = z.valid;
      out.push_back(row);
    }
    return out;
  }

  static void cancel(Node & n) {n.planner_timer_->cancel();}
  static void plan(Node & n, Memory & m, const StaticMap & sm, const Frames & fr, const J & cfg, const Points & route,
    const V & position, double yaw, double speed, double t, J & plans) {
    Node::GridMapSnapshot g; g.valid = true; g.frame_id = "Vehicle"; g.width = g.height = 128; g.resolution = .3125; g.origin_body = kGridOrigin; g.data.resize(16384);
    auto R = rotation(yaw);
    const bool reproj = cfg.value("grid", std::string("reproj")) == "reproj";
    int fi = reproj ? fr.nearest(position, yaw, cfg.value("exclude_attempt", -1)) : -1;
    Eigen::Matrix2d Rf; V pf;
    if (fi >= 0) {Rf = rotation(fr.pose[fi][2]); pf = V(fr.pose[fi][0], fr.pose[fi][1]);}
    for (int y = 0; y < 128; ++y) for (int x = 0; x < 128; ++x) {
      V w = position + R * (kGridOrigin + .3125 * V(x + .5, y + .5));
      int8_t v = static_cast<int8_t>(sm.cell(w));
      if (fi >= 0) {
        V b = Rf.transpose() * (w - pf) - kGridOrigin; int cx = floor(b.x() / .3125), cy = floor(b.y() / .3125);
        if (cx >= 0 && cy >= 0 && cx < 128 && cy < 128) v = fr.data[static_cast<size_t>(fi) * 16384 + cy * 128 + cx];
      }
      g.data[y * 128 + x] = v;
    }
    auto body = [&](const V & q) -> V {return R.transpose() * (q - position);};
    auto pair = egocentric_planner_geometry::findWaypointPair(route, m.wp, position, std::max(.01, speed) * V(cos(yaw), sin(yaw)), n.waypoint_switch_eps_m_, n.waypoint_switch_distance_m_); m.wp = pair[0];
    int horizon = n.computeRequestedPreviewSteps(&g, n.target_speed_mps_);
    horizon = egocentric_planner_geometry::retainMatlabPreviewHorizon(horizon, m.previous.size(), n.preview_restore_full_horizon_);
    Points prefix; bool used = m.previous.size() == static_cast<size_t>(horizon + 1) && egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(m.previous, position, yaw, n.target_speed_mps_ * n.upper_prediction_dt_sec_, horizon, prefix);
    if (!used) m.previous.clear();
    auto ref = n.buildPreviewReference(m.wp > 0 ? body(route[m.wp - 1]) : V::Zero(), body(route[m.wp]), body(route[pair[1]]), m.wp > 0, n.target_speed_mps_, horizon, prefix);
    auto pc = n.extractPreviewIntervals(ref, &g, m.interval); m.interval = pc.interval_update.state;
    n.applyTerminalPositionTargets(pc, body(route[m.wp]), body(route[pair[1]]), n.target_speed_mps_ * n.upper_prediction_dt_sec_);
    auto z = n.solveUpperMiqp(pc, n.target_speed_mps_);
    J out = {{"t", t}, {"x", position.x()}, {"y", position.y()}, {"yaw", yaw}, {"valid", z.valid}, {"status", z.status}, {"horizon", pc.N_pred}, {"counts", pc.Nck}, {"branch", z.branch_step}, {"selected", z.selected_corridors}, {"wp", m.wp}, {"B", m.interval.reference_length}, {"frame", fi},
      {"frame_attempt", fi >= 0 ? fr.pose[fi][3] : -1}, {"intervals", intervalsSummary(pc)}};
    if (z.valid) {
      Points world; for (const auto & q : z.predicted_body) world.push_back(position + R * q);
      std::vector<double> s; auto dense = n.densifyPath(world, n.target_speed_mps_, s); m.path = prepareLowerPath(dense, &s);
      m.previous = world.size() >= 3 ? world : Points{}; m.last_success = t; out["path"] = packed(world);
    }
    {J c = J::array(); for (size_t k = 0; k < pc.pmk.size(); ++k) {J r = J::array(); for (size_t i = 0; i < pc.pmk[k].size(); ++i) {V a = position + R * pc.pmk[k][i], b = position + R * pc.pMk[k][i]; r.push_back({a.x(), a.y(), b.x(), b.y()});} c.push_back(r);} out["corridors"] = c;
     J rf = J::array(); for (const auto & q : pc.prk) {V w = position + R * q; rf.push_back({w.x(), w.y()});} out["reference"] = rf;}
    plans.push_back(out);
  }
};
}  // namespace imac_ctrl

static J closedLoop(const J & cfg, const J & plant, const J & start, const StaticMap & sm, const Frames & fr, const std::string & params) {
  rclcpp::NodeOptions opt; opt.arguments({"--ros-args", "--params-file", params, "--log-level", "error"});
  opt.parameter_overrides({rclcpp::Parameter("mavlink_enable", false), rclcpp::Parameter("pixhawk_output_backend", "disabled"), rclcpp::Parameter("publish_pwm", false)});
  auto upper = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt); auto lower = std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);
  using UA = imac_ctrl::LowerPathGenerationTestAccess; using LA = imac_ctrl::LowerTrackingMpcTestAccess;
  UA::cancel(*upper); exp_reset();
  auto & impl = LA::get(*lower); impl.controller_timer_->cancel();
  if (impl.mavlinkActive()) throw std::runtime_error("hardware backend must be disabled");
  J rj; std::ifstream(start.value("route_file", std::string("route.json"))) >> rj; Points route = pts(rj["route"]);
  UA::Memory memory; memory.wp = start["wp"]; memory.interval.reference_length = start.value("B0", 0.0); J plans = J::array(), states = J::array();
  V xy(start["x"], start["y"]); double yaw = start["yaw"], delta = start.value("delta", 0.0), v = start["v"];
  int delay = std::lround(plant["delay"].get<double>() / .001); std::deque<double> pending(delay, start.value("delta", 0.0)); double tau = plant["tau"];
  int phase = cfg.value("phase", 0), period = 5;  // 2 Hz upper, 10 Hz lower
  std::string outcome = "time_limit"; int off = 0, far = 0;
  for (int step = 0; step < start.value("max_steps", 300); ++step) {
    double t = step * .1;
    if (step % period == phase || step == 0) UA::plan(*upper, memory, sm, fr, cfg, route, xy, yaw, v, t, plans);
    if (start.contains("end_x_gt") && xy.x() > start["end_x_gt"].get<double>()) {outcome = "east_done"; break;}
    if (start.contains("fail_y_gt") && xy.y() > start["fail_y_gt"].get<double>()) {outcome = "north_branch"; break;}
    if (start.contains("goal") && (xy - V(start["goal"][0], start["goal"][1])).norm() < start.value("goal_radius", 8.0)) {outcome = "goal"; break;}
    {
      double dmin = 1e30;
      for (size_t i = 0; i + 1 < route.size(); ++i) {V a = route[i], b = route[i + 1]; double u = std::clamp((xy - a).dot(b - a) / (b - a).squaredNorm(), 0.0, 1.0); dmin = std::min(dmin, (a + u * (b - a) - xy).norm());}
      far = dmin > start.value("off_route_m", 1e9) ? far + 1 : 0; if (far >= 10) {outcome = "off_route"; break;}
    }
    if (t - memory.last_success > 1.5) {outcome = "stale_path_stop"; break;}
    off = sm.cell(xy) > 50 ? off + 1 : 0; if (off >= 10) {outcome = "left_road"; break;}
    LA::Impl::PoseSnapshot pose; pose.position = xy; pose.yaw_rad = yaw; pose.speed_mps = v; pose.valid = pose.speed_valid = true; pose.frame_id = "map";
    auto z = impl.solveTrackingMpc(pose, memory.path);
    double command = z.valid ? z.steering_sequence_rad.front() : (z.fallback_valid ? z.fallback_steering_rad : 0.);
    double norm = impl.filterSteering(command, rclcpp::Time(static_cast<int64_t>((t + 1.) * 1e9), RCL_ROS_TIME)); double applied = norm * impl.steeringCommandScale();
    states.push_back({{"t", t}, {"x", xy.x()}, {"y", xy.y()}, {"yaw", yaw}, {"delta", delta}, {"cmd", applied}, {"lower_valid", z.valid}, {"ey", z.lateral_error_m}});
    for (int j = 0; j < 100; ++j) {
      double u = applied; if (delay) {u = pending.front(); pending.pop_front(); pending.push_back(applied);}
      delta += (1 - exp(-.001 / tau)) * (u - delta); double beta = atan(.5 * tan(delta)); double rate = v / 2.8 * cos(beta) * tan(delta);
      xy += .001 * v * V(cos(yaw + beta + .0005 * rate), sin(yaw + beta + .0005 * rate)); yaw = wrapToPi(yaw + .001 * rate);
    }
    impl.advanceSteeringActuatorModel(applied);
  }
  return J{{"config", cfg}, {"plant", plant}, {"start", start}, {"outcome", outcome}, {"plans", plans}, {"states", states}};
}

int main(int argc, char ** argv) {
  if (argc != 4) {std::cerr << "harness ol|cl INPUT_JSON OUTPUT_JSONL\n"; return 2;}
  std::string mode = argv[1]; J input; std::ifstream(argv[2]) >> input; rclcpp::init(0, nullptr); std::ofstream output(argv[3]);
  const std::string params = input["params"];
  if (mode == "ol") {
    J cycles; std::ifstream("ol_cycles.json") >> cycles; auto grids = readBin("ol_grids.bin");
    for (const auto & variant : input["variants"]) for (int attempt : input["attempts"].get<std::vector<int>>()) {
      exp_b_mode = variant["b_mode"]; exp_recover_cycles = variant.value("recover_cycles", 3);
      rclcpp::NodeOptions opt; opt.arguments({"--ros-args", "--params-file", params, "--log-level", "error"});
      auto node = std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);
      J r = {{"variant", variant["name"]}, {"attempt", attempt}, {"cycles", imac_ctrl::LowerPathGenerationTestAccess::openLoop(*node, cycles, grids, attempt)}};
      output << r.dump() << '\n'; output.flush(); std::cout << "ol " << variant["name"] << " " << attempt << std::endl;
    }
  } else {
    const std::string pre = input.value("prefix", std::string(""));
    StaticMap sm(pre + "static_map.bin", pre + "static_map.json"); Frames fr(pre + "cl_grids.bin", pre + "cl_poses.json");
    for (const auto & variant : input["variants"]) for (const auto & plant : input["plants"]) for (const auto & start : input["starts"])
      for (int phase = 0; phase < 5; ++phase) {
        J cfg = variant; cfg["phase"] = phase; exp_b_mode = variant["b_mode"]; exp_recover_cycles = variant.value("recover_cycles", 3);
        auto r = closedLoop(cfg, plant, start, sm, fr, params);
        output << r.dump() << '\n'; output.flush();
        std::cout << "cl " << variant["name"] << " " << plant["name"] << " " << start["name"] << " ph" << phase << " " << r["outcome"] << std::endl;
      }
  }
  rclcpp::shutdown(); return 0;
}
