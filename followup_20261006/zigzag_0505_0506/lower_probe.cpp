// Shadow numerical solves only: never spin an executor or publish commands.
#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "/home/imac/ros2_ws/src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include <nlohmann/json.hpp>
#include <fstream>
using J=nlohmann::json;
namespace imac_ctrl {
struct LowerTrackingMpcTestAccess {
  using Impl=LowerTrackingMpcNode::Impl;
  static J run(LowerTrackingMpcNode&node,const J&input) {
    auto& x=*node.impl_;x.controller_timer_->cancel();
    if(x.mavlinkActive() || x.publish_pwm_)throw std::runtime_error("hardware must be disabled");
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
      int i=r["path_index"].get<int>();J z={{"t",tr[0]},{"path_index",i},{"recorded_delta",tr[8]},{"age",r["age"]}};
      for(const auto&entry:std::vector<std::pair<std::string,int>>{{"current",i},{"previous",i-1}}){
        if(entry.second<0)continue;
        auto m=x.solveTrackingMpc(pose,paths[entry.second]);
        z[entry.first]={{"valid",m.valid},{"delta",m.valid?J(m.steering_sequence_rad.front()):J(nullptr)},
          {"lateral_error",m.lateral_error_m},{"heading_error",m.heading_error_rad},{"nearest",m.nearest_point}};
      }
      out.push_back(z);
    }
    return out;
  }
};
}
int main(int argc,char**argv){
  if(argc!=3)return 2;J input;std::ifstream(argv[1])>>input;
  rclcpp::init(0,nullptr);J out;
  for(auto it=input.begin();it!=input.end();++it){
    rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",it.value()["config"].get<std::string>(),"--log-level","error"});
    opt.parameter_overrides({rclcpp::Parameter("mavlink_enable",false),rclcpp::Parameter("pixhawk_output_backend","disabled"),rclcpp::Parameter("publish_pwm",false)});
    auto node=std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);
    out[it.key()]=imac_ctrl::LowerTrackingMpcTestAccess::run(*node,it.value());
  }
  std::ofstream(argv[2])<<out.dump(2)<<'\n';rclcpp::shutdown();
}
