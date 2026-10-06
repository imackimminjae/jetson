#include "virtual_control/tracking_control.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
namespace imac_ctrl {
struct LowerPathGenerationTestAccess {
 static void run(SdMapUpperPlannerNode & node, const char * filename) {
  node.planner_timer_->cancel();
  std::ifstream in(filename); int records; in >> records;
  for(int rec=0;rec<records;++rec) {
   SdMapUpperPlannerNode::PreviewConstraintData p;
   double stamp,yaw,speed; Eigen::Vector2d wp0,wp1;
   in >> stamp >> yaw >> p.N_pred >> speed >> p.road_segment_heading_rad[0]
      >> p.road_segment_heading_rad[1] >> p.delta_psi_road_rad
      >> wp0.x() >> wp0.y() >> wp1.x() >> wp1.y();
   const int n=p.N_pred;
   p.prk.resize(n+1); p.psirk.resize(n+1); p.Nck.resize(n+1);
   p.pmk.resize(n+1); p.pMk.resize(n+1);
   for(auto & v:p.prk) in >> v.x() >> v.y();
   for(auto & v:p.psirk) in >> v;
   for(auto & v:p.Nck) in >> v;
   for(int k=1;k<=n;++k) for(int j=0;j<p.Nck[k];++j) {
    Eigen::Vector2d lo,hi;in >> lo.x() >> lo.y() >> hi.x() >> hi.y();
    p.pmk[k].push_back(lo);p.pMk[k].push_back(hi);
   }
   node.applyTerminalPositionTargets(p,wp0,wp1,speed*node.upper_prediction_dt_sec_);
   for(int mode=0;mode<4;++mode) {
    node.upper_consistent_increment_model_=mode!=3;
    node.lambda_relative_turn_=mode==1 ? 0.0 : mode==2 ? 0.3 : 0.9;
    auto r=node.solveUpperMiqp(p,speed);
    std::cout << std::setprecision(17) << "SNAP {\"t\":" << stamp
      << ",\"mode\":" << mode << ",\"valid\":" << (r.valid?"true":"false")
      << ",\"yaw\":" << yaw << ",\"branch\":" << r.branch_step
      << ",\"costs\":[" << r.position_cost << ',' << r.input_cost << ',' << r.turn_cost << "]"
      << ",\"points_body\":[";
    for(size_t k=0;k<r.predicted_body.size();++k) {
      if(k)std::cout << ',';
      std::cout << '[' << r.predicted_body[k].x() << ',' << r.predicted_body[k].y() << ']';
    }
    std::cout << "]}" << std::endl;
   }
  }
 }
};
}
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 { auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>();
   imac_ctrl::LowerPathGenerationTestAccess::run(*node,argv[1]); }
 rclcpp::shutdown();
}
