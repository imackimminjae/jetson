#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
using J=nlohmann::json;
using V=Eigen::Vector2d;
using M=Eigen::Matrix2d;
static V vec(const J &x){return V(x.at(0).get<double>(),x.at(1).get<double>());}
static M rot(double a){M r;r<<cos(a),-sin(a),sin(a),cos(a);return r;}
static std::vector<V> points(const J &x){std::vector<V> p;for(const auto &q:x)p.push_back(vec(q));return p;}
static J coords(const std::vector<V>&p){J j=J::array();for(const auto&q:p)j.push_back({q.x(),q.y()});return j;}

namespace imac_ctrl {
struct LowerPathGenerationTestAccess {
 using Node=SdMapUpperPlannerNode;
 using C=Node::PreviewConstraintData;
 static void cancel(Node &n){n.planner_timer_->cancel();}
 static J run(Node &n,const J&catalog,const J&job){
  int si=job.at("s");const auto&s=catalog.at("snapshots").at(si);const auto&e=s.at("event");
  int pi=job.value("pose_s",si),wi=job.value("wp_s",si),bi=job.value("B_s",si),vi=job.value("previous_s",si);
  const auto&pe=catalog["snapshots"][pi]["event"];const auto&we=catalog["snapshots"][wi]["event"];
  V origin(pe["x"].get<double>(),pe["y"].get<double>());double yaw=pe["yaw_rad"];M R=rot(yaw);
  auto body=[&](const V&p)->V{return R.transpose()*(p-origin);};
  const auto &route=catalog["route"];int wp0=we["wp0"],wp1=we["wp1"];
  V w0=body(vec(route[wp0])),w1=body(vec(route[wp1]));
  C c;J result;result["id"]=job["id"];result["s"]=si;result["clock"]=s["clock"];
  n.upper_r_dpsi_=job.value("dpsi_weight",1.0);
  n.lambda_relative_turn_=job.value("turn_weight",.6);
  if(job["mode"]=="recorded"){
   c.N_pred=e["horizon"];int size=c.N_pred+1;
   for(const auto&p:e["preview_world"])c.prk.push_back(body(vec(p)));
   for(int k=0;k<size;++k)c.psirk.push_back(s["trace"][18+k]);
   c.pmk.resize(size);c.pMk.resize(size);c.raw_lengths.resize(size);c.processed_lengths.resize(size);c.source_modes.resize(size);c.treatment_modes.resize(size);c.fallback_reason_codes.resize(size);
   c.Nck=e["candidate_counts"].get<std::vector<int>>();
   c.road_segment_heading_rad={s["trace"][11].get<double>(),s["trace"][12].get<double>()};c.delta_psi_road_rad=s["trace"][13];
   for(const auto&i:e["intervals_world"]){int k=i[0];c.pmk[k].push_back(body(V(i[2].get<double>(),i[3].get<double>())));c.pMk[k].push_back(body(V(i[4].get<double>(),i[5].get<double>())));c.raw_lengths[k].push_back(i[6]);c.processed_lengths[k].push_back(i[7]);c.source_modes[k].push_back(i[8]);c.fallback_reason_codes[k]=i[9];}
  }else{
   Node::GridMapSnapshot grid;const auto&g=catalog["grids"][std::to_string(job.at("grid").get<int>())];
   grid.valid=true;grid.frame_id="Vehicle";grid.width=g["width"];grid.height=g["height"];grid.resolution=g["res"];grid.origin_body=vec(g["origin"]);grid.origin_yaw_rad=0;
   for(const auto&v:g["data"])grid.data.push_back(static_cast<int8_t>(v.get<int>()));
   // Express a frozen source grid in the tested pose's body frame without
   // resampling its cells; changing pose alone must not move the world road.
   int gi=job.value("grid_frame_s",pi);
   if(gi!=pi){const auto&ge=catalog["snapshots"][gi]["event"];double gy=ge["yaw_rad"];V go(ge["x"].get<double>(),ge["y"].get<double>());grid.origin_body=body(go+rot(gy)*grid.origin_body);grid.origin_yaw_rad=gy-yaw;}
   int horizon=job.value("horizon",7);
   std::vector<V> prefix;
   auto previous=points(catalog["snapshots"][vi]["previous_world"]);
   if(job.value("no_previous",false))previous.clear();
   if(previous.size()==static_cast<size_t>(horizon+1))egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(previous,origin,yaw,4.5,horizon,prefix);
   auto ref=n.buildPreviewReference(wp0>0?body(vec(route[wp0-1])):V::Zero(),w0,w1,wp0>0,6.0,horizon,prefix);
   Node::IntervalCenteringState state;state.reference_length=catalog["snapshots"][bi]["B_before"];
   c=n.extractPreviewIntervals(ref,&grid,state);
   result["B_after"]=c.interval_update.state.reference_length;result["used_previous"]=ref.used_previous_solution;
  }
  if(job.contains("prune_unselected_below")){
   double threshold=job["prune_unselected_below"];J removed=J::array();
   for(int k=1;k<=c.N_pred;++k){
    int selected=e["valid_plan"].get<bool>()?e["selected_corridors"][k].get<int>():-1;
    for(int j=static_cast<int>(c.pmk[k].size())-1;j>=0;--j){
     if(j==selected || c.raw_lengths[k][j]>=threshold)continue;
     removed.push_back({k,j,c.raw_lengths[k][j]});
     c.pmk[k].erase(c.pmk[k].begin()+j);c.pMk[k].erase(c.pMk[k].begin()+j);
     c.raw_lengths[k].erase(c.raw_lengths[k].begin()+j);c.processed_lengths[k].erase(c.processed_lengths[k].begin()+j);
    }
    c.Nck[k]=static_cast<int>(c.pmk[k].size());
   }
   result["removed_unselected"]=removed;
  }
  n.applyTerminalPositionTargets(c,w0,w1,4.5);
  result["horizon"]=c.N_pred;result["counts"]=c.Nck;result["reference_body"]=coords(c.prk);result["headings"]=c.psirk;result["target_body"]=coords(c.position_targets);
  J intervals=J::array();for(int k=1;k<=c.N_pred;++k)for(size_t j=0;j<c.pmk[k].size();++j){V a=origin+R*c.pmk[k][j],b=origin+R*c.pMk[k][j];intervals.push_back({k,j,a.x(),a.y(),b.x(),b.y(),c.raw_lengths[k][j],c.processed_lengths[k][j]});}result["intervals_world"]=intervals;
  if(job.value("no_solve",false))return result;
  int repeats=job.value("repeat",1);J trials=J::array();
  for(int i=0;i<repeats;++i){
   auto z=n.solveUpperMiqp(c,6.0);J t;t["valid"]=z.valid;t["status"]=z.status;t["cost"]={z.position_cost,z.input_cost,z.turn_cost};t["branch_step"]=z.branch_step;t["selected"]=z.selected_corridors;t["solve_ms"]=z.solver_only_time_ms;
   std::vector<V>w;for(const auto&q:z.predicted_body)w.push_back(origin+R*q);t["world"]=coords(w);t["correction"]=z.correction_input;
   if(w.size()>1)t["h1_deg"]=atan2((w[1]-w[0]).y(),(w[1]-w[0]).x())*180/M_PI;
   trials.push_back(t);
  }
  result["trials"]=trials;return result;
 }
};
}
int main(int argc,char**argv){
 if(argc<4){std::cerr<<"usage: probe INPUT OUTPUT CONFIG\n";return 2;}
 J input;std::ifstream(argv[1])>>input;
 rclcpp::init(0,nullptr);
 rclcpp::NodeOptions options;options.arguments({"--ros-args","--params-file",argv[3],"--log-level","error"});
 auto n=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);imac_ctrl::LowerPathGenerationTestAccess::cancel(*n);
 std::ofstream out(argv[2]);
 for(const auto&job:input["jobs"]){try{
  if(job.contains("feedback_cycles")){
   J working=input,series=J::array();int si=job["s"];
   for(int k=0;k<job["feedback_cycles"].get<int>();++k){
    auto r=imac_ctrl::LowerPathGenerationTestAccess::run(*n,working,job);r["iteration"]=k;series.push_back(r);
    if(r.contains("B_after"))working["snapshots"][si]["B_before"]=r["B_after"];
    if(r["trials"][0]["valid"].get<bool>())working["snapshots"][si]["previous_world"]=r["trials"][0]["world"];
   }
   out<<J({{"id",job["id"]},{"series",series}}).dump()<<'\n';
  }else{out<<imac_ctrl::LowerPathGenerationTestAccess::run(*n,input,job).dump()<<'\n';}
  out.flush();
 }catch(const std::exception&e){out<<J({{"id",job["id"]},{"error",e.what()}}).dump()<<'\n';}}
 n.reset();rclcpp::shutdown();return 0;
}
