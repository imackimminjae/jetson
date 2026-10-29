#include "virtual_control/comparison_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <imac_interfaces/msg/path_with_arc_length.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <sstream>
#include <stdexcept>

namespace imac_ctrl::comparison
{
namespace
{
std::string frame(std::string s)
{while (!s.empty() && s.front()=='/') {s.erase(s.begin());} return s;}
double yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if (!std::isfinite(norm) || norm<1e-10 || std::abs(norm-1)>0.01) {
    throw std::invalid_argument("invalid quaternion");
  }
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
std::string jsonString(const std::string & s)
{
  std::string result="\"";
  for (unsigned char ch : s) {
    if (ch=='"' || ch=='\\') {result+='\\'; result+=ch;}
    else if (ch>=32) {result+=ch;}
  }
  return result+'"';
}
}  // namespace
// The default single-threaded executor serializes all state and solve callbacks.
class ComparisonPlannerNode : public rclcpp::Node
{
public:
  explicit ComparisonPlannerNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions(),
    const std::string & node_name="comparison_planner_node")
  : Node(node_name,options)
  {
    planner_=declare_parameter<std::string>("planner","nvp");
    reference_source_=declare_parameter<std::string>("reference_source","exact");
    if (planner_!="nvp" && planner_!="mpcc") {throw std::invalid_argument("planner must be nvp or mpcc");}
    if (reference_source_!="exact" && reference_source_!="sparse") {
      throw std::invalid_argument("reference_source must be exact or sparse");
    }
    if (planner_=="nvp") {reference_source_="sparse";}
    world_=frame(declare_parameter<std::string>("path_frame_id","map"));
    ego_=frame(declare_parameter<std::string>("expected_grid_frame_id","Vehicle"));
    scalar_yaw_=declare_parameter<bool>("odom_yaw_is_orientation_z",true);
    bias_enabled_=declare_parameter<bool>("enable_waypoint_bias",false);
    bias_={declare_parameter<double>("waypoint_x_bias_m",0.0),
      declare_parameter<double>("waypoint_y_bias_m",0.0)};
    bias_yaw_=declare_parameter<double>("waypoint_yaw_bias_rad",0.0);
    if (planner_=="mpcc" && reference_source_=="exact" && bias_enabled_) {
      throw std::invalid_argument("exact reference mode requires waypoint bias disabled");
    }
    timeout_=declare_parameter<double>("input_timeout_sec",0.6);
    skew_=declare_parameter<double>("max_grid_pose_skew_sec",0.1);
    pose_jump_=declare_parameter<double>("pose_jump_reset_threshold_m",1.0);
    steering_timeout_=declare_parameter<double>("steering_timeout_sec",0.25);
    auto & c=input_.settings; auto & v=input_.geometry;
    c.period=1.0/declare_parameter<double>("upper_planner_rate_hz",5.0);
    c.dt=declare_parameter<double>("upper_prediction_dt_sec",0.5);
    c.steps=declare_parameter<int>("upper_preview_steps",7);
    c.prediction_speed=declare_parameter<double>("target_speed_mps",6.0);
    c.lower_dt=declare_parameter<double>("lower_prediction_dt_sec",0.1);
    c.min_spacing=declare_parameter<double>("lower_min_path_spacing_m",0.05);
    c.min_length=declare_parameter<double>("min_path_length_m",1.0);
    v.wheelbase=declare_parameter<double>("wheelbase",2.8);
    v.length=declare_parameter<double>("vehicle_length_m",4.7);
    v.width=declare_parameter<double>("vehicle_width_m",1.8);
    v.reference_to_rear=declare_parameter<double>("reference_point_to_rear_axle_m",1.4);
    v.max_steer=declare_parameter<double>("max_steering_angle_rad",0.30);
    v.max_steer_rate=declare_parameter<double>("max_steering_rate_radps",1.0471975512);
    c.extra_margin=declare_parameter<double>("extra_road_margin_m",1.1);
    c.allow_initial_unobserved_footprint=declare_parameter<bool>("allow_initial_unobserved_footprint",true);
    c.corridor_scan=declare_parameter<double>("corridor_scan_m",10.0);
    c.projection_back=declare_parameter<double>("projection_back_m",1.0);
    c.projection_slack=declare_parameter<double>("projection_slack_m",2.0);
    const double seed=declare_parameter<double>("initial_progress_m",0.0);
    initial_progress_=seed<0 ? std::nullopt : std::optional<double>(seed);
    input_.progress=initial_progress_;
    c.ring_step=declare_parameter<double>("nvp_ring_step_m",1.0);
    c.angular_step=declare_parameter<double>("nvp_angular_step_rad",0.01);
    c.arc_half_angle=declare_parameter<double>("nvp_arc_half_angle_rad",1.50);
    c.valley_gradient=declare_parameter<double>("nvp_valley_gradient",0.12);
    c.attraction=declare_parameter<double>("nvp_attraction_weight",1.0);
    c.repulsion=declare_parameter<double>("nvp_repulsion_weight",1.0);
    c.attraction_power=declare_parameter<double>("nvp_attraction_power",1.0);
    c.repulsion_power=declare_parameter<double>("nvp_repulsion_power",1.0);
    c.max_connection=declare_parameter<double>("nvp_max_connection_m",3.0);
    c.sqp_iterations=declare_parameter<int>("mpcc_sqp_iterations",6);
    c.q_contour=declare_parameter<double>("mpcc_q_contour",10.0);
    c.q_lag=declare_parameter<double>("mpcc_q_lag",20.0);
    c.reward_progress=declare_parameter<double>("mpcc_progress_reward",2.0);
    c.r_steer=declare_parameter<double>("mpcc_r_steer",0.3);
    c.r_steer_change=declare_parameter<double>("mpcc_r_steer_change",15.0);
    c.max_virtual_speed=declare_parameter<double>("mpcc_max_virtual_speed_mps",9.0);
    c.steer_trust=declare_parameter<double>("mpcc_steer_trust_rad",0.12);
    c.progress_trust=declare_parameter<double>("mpcc_progress_trust_m",2.0);
    input_.grid.positive_is_drivable=declare_parameter<bool>("grid_positive_is_drivable",false);
    input_.grid.threshold=declare_parameter<int>("grid_value_threshold",0);
    if (!std::isfinite(c.period) || c.period<=0 || timeout_<=0 || skew_<0 ||
      steering_timeout_<=0 || pose_jump_<=0 || world_.empty() || ego_.empty()) {
      throw std::invalid_argument("invalid timing/frame configuration");
    }
    const auto odom=declare_parameter<std::string>("odom_topic","/motive/vehicle/odom_map");
    const auto bev=declare_parameter<std::string>("grid_map_topic","/bev/occupancy_grid");
    const auto route=declare_parameter<std::string>("global_path_topic","/navigation/global_path");
    const auto exact=declare_parameter<std::string>("exact_reference_topic","/navigation/exact_reference_path");
    const auto output=declare_parameter<std::string>("lower_reference_path_topic","/planner/lower_reference_path");
    const auto sparse=declare_parameter<std::string>("upper_sparse_path_topic","/planner/upper_path_sparse");
    const auto goal=declare_parameter<std::string>("goal_reached_topic","/planner/goal_reached");
    const auto reset=declare_parameter<std::string>("reset_topic","/controller/reset");
    auto latched=rclcpp::QoS(1).reliable().transient_local();
    path_pub_=create_publisher<nav_msgs::msg::Path>(output,latched);
    sparse_pub_=create_publisher<nav_msgs::msg::Path>(sparse,latched);
    arc_pub_=create_publisher<imac_interfaces::msg::PathWithArcLength>(output+"/with_arclength",latched);
    goal_pub_=create_publisher<std_msgs::msg::Bool>(goal,latched);
    status_pub_=create_publisher<std_msgs::msg::String>("/debug/comparison_planner_status",10);
    timing_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/debug/comparison_planner_timing",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odom,10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {acceptOdom(*msg);});
    grid_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(bev,10,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        grid_message_=msg; grid_received_=now().seconds();
      });
    // Exact MPCC gets a dedicated externally supplied reference, never MIQP output.
    route_sub_=create_subscription<nav_msgs::msg::Path>(
      planner_=="mpcc" && reference_source_=="exact" ? exact : route,latched,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {acceptRoute(*msg);});
    steer_sub_=create_subscription<std_msgs::msg::Float64MultiArray>(
      "/debug/lower_mpc_trace",rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size()>20 && std::isfinite(msg->data[0]) && std::isfinite(msg->data[20])) {
          steering_time_=msg->data[0]; applied_=msg->data[20];
        }
      });
    reset_sub_=create_subscription<std_msgs::msg::Bool>(reset,10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {if (msg->data) {resetMission();}});
    reset_service_=create_service<std_srvs::srv::Trigger>("~/reset_mission",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        resetMission(); response->success=true; response->message="comparison planner reset";
      });
    timer_=create_wall_timer(std::chrono::duration<double>(c.period),[this]() {tick();});
    RCLCPP_INFO(get_logger(),"%s + common tracking MPC; reference_source=%s; fixed prediction speed=%.2f",
      planner_.c_str(),reference_source_.c_str(),c.prediction_speed);
  }
private:
  struct StateSample {double stamp; Pose pose; double speed;};
  double messageTime(const builtin_interfaces::msg::Time & stamp,double receipt) const
  {
    return stamp.sec==0 && stamp.nanosec==0 ? receipt :
      rclcpp::Time(stamp,get_clock()->get_clock_type()).seconds();
  }
  bool fresh(double stamp,double time) const {return time-stamp>=-0.05 && time-stamp<=timeout_;}
  void resetMission()
  {
    nvp_.reset(); mpcc_.reset(); input_.progress=initial_progress_;
    applied_.reset(); goal_=false;
    Output out; out.reason="mission reset"; publish(out,now());
  }
  void acceptOdom(const nav_msgs::msg::Odometry & msg)
  {
    const double receipt=now().seconds();
    try {
      if (frame(msg.header.frame_id)!=world_) {throw std::runtime_error("odometry frame mismatch");}
      StateSample state;
      state.stamp=messageTime(msg.header.stamp,receipt);
      state.pose={{msg.pose.pose.position.x,msg.pose.pose.position.y},
        scalar_yaw_ ? msg.pose.pose.orientation.z : yaw(msg.pose.pose.orientation)};
      state.speed=std::hypot(msg.twist.twist.linear.x,msg.twist.twist.linear.y);
      if (!state.pose.p.allFinite() || !std::isfinite(state.pose.yaw) || !std::isfinite(state.speed) ||
        !fresh(state.stamp,receipt)) {throw std::runtime_error("invalid/stale odometry");}
      if (!history_.empty() && state.stamp<=history_.back().stamp) {return;}
      if (!history_.empty() && (state.pose.p-history_.back().pose.p).norm()>pose_jump_) {
        resetMission(); history_.clear(); grid_message_.reset();
      }
      history_.push_back(state);
      while (history_.size()>300 || history_.front().stamp<state.stamp-3.0) {history_.pop_front();}
      odom_received_=receipt; odom_error_.clear();
    } catch (const std::exception & e) {odom_error_=e.what(); history_.clear();}
  }
  void acceptRoute(const nav_msgs::msg::Path & msg)
  {
    try {
      if (frame(msg.header.frame_id)!=world_) {throw std::runtime_error("reference frame mismatch");}
      std::vector<Point> points;
      for (const auto & pose : msg.poses) {
        if ((!pose.header.frame_id.empty() && frame(pose.header.frame_id)!=world_) ||
          !std::isfinite(pose.pose.position.z)) {throw std::runtime_error("invalid reference pose frame");}
        Point p(pose.pose.position.x,pose.pose.position.y);
        if (!p.allFinite()) {throw std::runtime_error("nonfinite reference");}
        points.push_back(p);
      }
      ReferenceCurve checked(points);
      if (bias_enabled_) {
        const Point anchor=points.front();
        for (auto & p : points) {p=anchor+rotate(p-anchor,bias_yaw_)+bias_;}
      }
      bool same=points.size()==input_.route.size();
      for (size_t i=0; same && i<points.size(); ++i) {same=(points[i]-input_.route[i]).norm()<1e-8;}
      if (!same) {resetMission(); input_.route=std::move(points);}
      route_error_.clear();
    } catch (const std::exception & e) {input_.route.clear(); route_error_=e.what(); resetMission();}
  }
  Pose poseAt(double stamp) const
  {
    if (history_.empty()) {throw std::runtime_error("no grid acquisition pose");}
    auto next=std::lower_bound(history_.begin(),history_.end(),stamp,
      [](const StateSample & a,double b) {return a.stamp<b;});
    if (next==history_.begin() || next==history_.end()) {
      const auto & sample=next==history_.end() ? history_.back() : history_.front();
      if (std::abs(sample.stamp-stamp)>skew_) {throw std::runtime_error("grid/pose timestamp skew");}
      return sample.pose;
    }
    const auto & a=*(next-1); const auto & b=*next;
    if (b.stamp-a.stamp>2*skew_) {throw std::runtime_error("grid pose history gap");}
    const double t=(stamp-a.stamp)/(b.stamp-a.stamp);
    const double angle=std::atan2(std::sin(b.pose.yaw-a.pose.yaw),std::cos(b.pose.yaw-a.pose.yaw));
    return {a.pose.p+t*(b.pose.p-a.pose.p),a.pose.yaw+t*angle};
  }
  void tick()
  {
    const auto wall_start=std::chrono::steady_clock::now();
    const auto stamp=now(); const double time=stamp.seconds(); Output out;
    try {
      if (last_cycle_ && time<*last_cycle_) {
        resetMission(); history_.clear(); grid_message_.reset();
      }
      last_cycle_=time;
      if (history_.empty() || !fresh(odom_received_,time) || !fresh(history_.back().stamp,time)) {
        throw std::runtime_error(odom_error_.empty() ? "missing/stale odometry" : odom_error_);
      }
      if (input_.route.size()<2) {
        throw std::runtime_error(route_error_.empty() ? "waiting for reference input" : route_error_);
      }
      if (!grid_message_ || !fresh(grid_received_,time)) {throw std::runtime_error("missing/stale BEV");}
      const auto & msg=*grid_message_;
      const double grid_stamp=messageTime(msg.header.stamp,grid_received_);
      if (!fresh(grid_stamp,time)) {throw std::runtime_error("stale BEV acquisition time");}
      auto & grid=input_.grid;
      if (frame(msg.header.frame_id)==world_) {grid.frame_world=Pose{};}
      else if (frame(msg.header.frame_id)==ego_) {grid.frame_world=poseAt(grid_stamp);}
      else {throw std::runtime_error("BEV frame mismatch (no implicit transform)");}
      grid.width=static_cast<int>(msg.info.width); grid.height=static_cast<int>(msg.info.height);
      grid.resolution=msg.info.resolution;
      grid.origin={msg.info.origin.position.x,msg.info.origin.position.y};
      grid.origin_yaw=yaw(msg.info.origin.orientation);
      if (!std::isfinite(msg.info.origin.position.z) || std::abs(msg.info.origin.orientation.x)>1e-6 ||
        std::abs(msg.info.origin.orientation.y)>1e-6) {throw std::runtime_error("nonplanar BEV origin");}
      grid.data=msg.data;
      if (!grid.valid()) {throw std::runtime_error("invalid BEV dimensions/data");}
      input_.vehicle=history_.back().pose; input_.actual_speed=history_.back().speed; input_.time=time;
      input_.applied_steer=applied_ && time-steering_time_>=-0.05 &&
        time-steering_time_<=steering_timeout_ ? applied_ : std::nullopt;
      out=planner_=="nvp" ? nvp_.plan(input_) : mpcc_.plan(input_);
      goal_=goal_ || out.goal_reached; out.goal_reached=goal_;
    } catch (const std::exception & error) {out.reason=error.what();}
    out.goal_reached=goal_ || out.goal_reached;
    const double before_publish=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-wall_start).count();
    out.preprocessing_ms+=std::max(0.0,before_publish-out.total_ms);
    publish(out,stamp);
    const double full=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-wall_start).count();
    out.postprocessing_ms+=full-before_publish; out.total_ms=full;
    report(out,stamp);
  }
  nav_msgs::msg::Path message(const std::vector<Point> & points,const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path msg; msg.header.frame_id=world_; msg.header.stamp=stamp;
    for (size_t i=0; i<points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose; pose.header=msg.header;
      pose.pose.position.x=points[i].x(); pose.pose.position.y=points[i].y();
      const Point d=points[std::min(i+1,points.size()-1)]-points[i==points.size()-1 && i>0 ? i-1 : i];
      const double angle=std::atan2(d.y(),d.x());
      pose.pose.orientation.z=std::sin(angle/2); pose.pose.orientation.w=std::cos(angle/2);
      msg.poses.push_back(pose);
    }
    return msg;
  }
  void publish(const Output & out,const rclcpp::Time & stamp)
  {
    imac_interfaces::msg::PathWithArcLength bundle;
    // Empty atomic snapshots invalidate the lower path immediately on failure.
    bundle.path=message(out.success ? out.path : std::vector<Point>{},stamp);
    if (out.success) {bundle.s=out.s;}
    arc_pub_->publish(bundle); path_pub_->publish(bundle.path);
    sparse_pub_->publish(message(out.success ? out.sparse : std::vector<Point>{},stamp));
    std_msgs::msg::Bool goal; goal.data=goal_ || out.goal_reached; goal_pub_->publish(goal);
  }
  void report(const Output & out,const rclcpp::Time & stamp)
  {
    std_msgs::msg::Float64MultiArray timing;
    timing.data={stamp.seconds(),out.preprocessing_ms,out.planning_ms,out.postprocessing_ms,
      out.total_ms,out.success ? 1.0 : 0.0}; timing_pub_->publish(timing);
    std::ostringstream json;
    json<<"{\"stamp\":"<<std::fixed<<stamp.seconds()<<",\"planner\":"<<jsonString(planner_)
      <<",\"reference_source\":"<<jsonString(planner_=="nvp" ? "sparse" : reference_source_)
      <<",\"success\":"<<(out.success ? "true" : "false")
      <<",\"goal_reached\":"<<(out.goal_reached ? "true" : "false")
      <<",\"reason\":"<<jsonString(out.reason)<<",\"actual_speed\":"<<input_.actual_speed
      <<",\"prediction_speed\":"<<input_.settings.prediction_speed
      <<",\"applied_steer_available\":"<<(input_.applied_steer ? "true" : "false")
      <<",\"projected_s\":"<<out.projected_s<<",\"warm_shift_sec\":"<<out.warm_shift_sec
      <<",\"qp_status\":"<<out.qp_status<<",\"iterations\":"<<out.iterations
      <<",\"candidates\":"<<out.candidates<<",\"max_constraint_violation\":"<<out.max_constraint_violation
      <<",\"preprocessing_ms\":"<<out.preprocessing_ms<<",\"planning_ms\":"<<out.planning_ms
      <<",\"postprocessing_ms\":"<<out.postprocessing_ms<<",\"total_ms\":"<<out.total_ms<<"}";
    std_msgs::msg::String status; status.data=json.str(); status_pub_->publish(status);
  }
  Input input_; NvpPlanner nvp_; MpccPlanner mpcc_;
  std::string planner_,reference_source_,world_,ego_,odom_error_,route_error_;
  bool scalar_yaw_{true},goal_{false};
  bool bias_enabled_{false}; Point bias_{0,0}; double bias_yaw_{0};
  double timeout_{0.6},skew_{0.1},pose_jump_{1},steering_timeout_{0.25};
  double odom_received_{0},grid_received_{0},steering_time_{0};
  std::optional<double> applied_,initial_progress_,last_cycle_;
  std::deque<StateSample> history_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_message_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_,sparse_pub_;
  rclcpp::Publisher<imac_interfaces::msg::PathWithArcLength>::SharedPtr arc_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
std::shared_ptr<rclcpp::Node> makePlannerNode(const std::string & planner)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("planner",planner);
  return std::make_shared<ComparisonPlannerNode>(options,planner+"_planner_node");
}
}  // namespace imac_ctrl::comparison
