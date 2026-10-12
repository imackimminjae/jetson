// Offline numerical calls only. No executor spin, publisher calls, or hardware outputs.
#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "/home/imac/ros2_ws/src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include <nlohmann/json.hpp>
#include <fstream>
using J=nlohmann::json;
namespace imac_ctrl {
struct LowerTrackingMpcTestAccess {
  using Impl=LowerTrackingMpcNode::Impl;
  static J run(LowerTrackingMpcNode&node,const J&input) {
    auto&x=*node.impl_;x.controller_timer_->cancel();
    if(x.mavlinkActive()||x.publish_pwm_||x.enable_output_filter_)throw std::runtime_error("require disabled hardware and recorded output filter=false");
    std::vector<LowerPathGeometry> paths;
    for(const auto&p:input["paths"]){
      std::vector<Eigen::Vector2d> xy;for(const auto&q:p["xy"])xy.emplace_back(q[0].get<double>(),q[1].get<double>());
      auto s=p["s"].get<std::vector<double>>();paths.push_back(prepareLowerPath(xy,&s));
    }
    J out=J::array();
    for(const auto&r:input["samples"]){
      auto tr=r["trace"].get<std::vector<double>>();
      Impl::PoseSnapshot pose;pose.valid=pose.speed_valid=true;pose.position={tr[1],tr[2]};pose.yaw_rad=tr[3];pose.speed_mps=tr[4];
      x.last_applied_steering_angle_rad_=tr[29];x.estimated_effective_steering_angle_rad_=tr[19];
      J z={{"t",tr[0]},{"cycle",r["cycle"]},{"x",tr[1]},{"y",tr[2]},{"recorded_delta",tr[8]},{"recorded_output",tr[26]}};
      for(auto it=r["paths"].begin();it!=r["paths"].end();++it){
        auto m=x.solveTrackingMpc(pose,paths[it.value().get<int>()]);
        const bool available=m.valid||m.fallback_valid;
        double delta=m.valid?m.steering_sequence_rad.front():m.fallback_steering_rad;
        // Production output filter is disabled. Preserve float quantization of VirtualControlCommand.steer.
        const double output=available?static_cast<double>(static_cast<float>(x.filterSteering(delta,rclcpp::Time(static_cast<int64_t>(tr[0]*1e9)))))*x.steeringCommandScale():0.0;
        z[it.key()]={{"valid",m.valid},{"fallback_valid",m.fallback_valid},{"delta",available?J(delta):J(nullptr)},{"output",available?J(output):J(nullptr)},{"lateral",m.lateral_error_m},{"heading",m.heading_error_rad}};
      }
      out.push_back(z);
    }
    return out;
  }
};
}
int main(int argc,char**argv){
  if(argc!=4)return 2;J input;std::ifstream(argv[1])>>input;
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
  opt.parameter_overrides({rclcpp::Parameter("mavlink_enable",false),rclcpp::Parameter("pixhawk_output_backend","disabled"),rclcpp::Parameter("publish_pwm",false)});
  auto node=std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);
  std::ofstream(argv[2])<<imac_ctrl::LowerTrackingMpcTestAccess::run(*node,input).dump();
  rclcpp::shutdown();return 0;
}
