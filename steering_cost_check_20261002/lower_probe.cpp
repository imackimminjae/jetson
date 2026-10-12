#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "../src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include <nlohmann/json.hpp>
#include <fstream>
using J=nlohmann::json;
namespace imac_ctrl {struct LowerTrackingMpcTestAccess {using Impl=LowerTrackingMpcNode::Impl;static Impl&get(LowerTrackingMpcNode&n){return *n.impl_;}};}
int main(int argc,char**argv){
 if(argc!=4)return 2;J input;std::ifstream(argv[1])>>input;rclcpp::init(0,nullptr);
 rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
 opt.parameter_overrides({rclcpp::Parameter("mavlink_enable",false),rclcpp::Parameter("pixhawk_output_backend","disabled"),rclcpp::Parameter("publish_pwm",false)});
 auto node=std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);auto&impl=imac_ctrl::LowerTrackingMpcTestAccess::get(*node);impl.controller_timer_->cancel();if(impl.mavlinkActive())throw std::runtime_error("hardware enabled");
 std::vector<imac_ctrl::LowerPathGeometry> paths;
 for(const auto&r:input["refs"]){std::vector<Eigen::Vector2d> xy;for(const auto&q:r["xy"])xy.emplace_back(q[0].get<double>(),q[1].get<double>());auto s=r["s"].get<std::vector<double>>();paths.push_back(imac_ctrl::prepareLowerPath(xy,&s));}
 std::ofstream out(argv[2]);for(const auto&r:input["rows"]){auto a=r["trace"].get<std::vector<double>>();imac_ctrl::LowerTrackingMpcTestAccess::Impl::PoseSnapshot pose;pose.position={a[1],a[2]};pose.yaw_rad=a[3];pose.speed_mps=a[4];pose.valid=pose.speed_valid=true;pose.frame_id="map";
 impl.estimated_effective_steering_angle_rad_=a[19];impl.last_applied_steering_angle_rad_=a[29];
 J result={{"t",r["t"]},{"clock",r["clock"]},{"recorded",a[25]},{"trials",J::array()}};
 for(double w:{60.,120.,240.}){impl.lower_rd_steering_rate_=w;auto z=impl.solveTrackingMpc(pose,paths[r["ref"].get<int>()]);result["trials"].push_back({{"weight",w},{"valid",z.valid},{"steering",z.valid?z.steering_sequence_rad.front():0.},{"ey",z.lateral_error_m},{"epsi",z.heading_error_rad}});}out<<result.dump()<<'\n';}
 node.reset();rclcpp::shutdown();
}
