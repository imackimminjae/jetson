// Standalone numerical harness. No spinning, command publication, or hardware I/O.
#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "/home/imac/ros2_ws/src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json; extern double g_soft_max_len;

using V=Eigen::Vector2d;
using Points=std::vector<V>;
static Points pts(const J&j){Points p;for(const auto&q:j)p.emplace_back(q[0].get<double>(),q[1].get<double>());return p;}
static J packed(const Points&p){J j=J::array();for(const auto&q:p)j.push_back({q.x(),q.y()});return j;}
static Eigen::Matrix2d rotation(double a){Eigen::Matrix2d r;r<<cos(a),-sin(a),sin(a),cos(a);return r;}
struct World {
 J scene;Points route,center,spur;std::vector<double> arc,widths;std::vector<int8_t> cells;V origin;int w,h;double res;
 explicit World(const J&s):scene(s),route(pts(s["route"])),center(pts(s["center"])),spur(pts(s["spur"])),arc(s["arc"].get<std::vector<double>>()),widths(s["widths"].get<std::vector<double>>()) {
  origin=V(s["origin"][0],s["origin"][1]);w=s["width"];h=s["height"];res=s["resolution"];cells.resize(w*h);std::ifstream f(s["bitmap"].get<std::string>(),std::ios::binary);f.read(reinterpret_cast<char*>(cells.data()),cells.size());if(!f)throw std::runtime_error("bad world bitmap");
 }
 int cell(const V&q)const{int x=floor((q.x()-origin.x())/res),y=floor((q.y()-origin.y())/res);return x>=0&&y>=0&&x<w&&y<h?cells[y*w+x]:100;}
 J measure(const V&q)const{
  size_t best=0;double distance=1e30;for(size_t i=0;i<center.size();++i){double d=(q-center[i]).squaredNorm();if(d<distance){distance=d;best=i;}}
  double margin=widths[best]/2-sqrt(distance);for(const auto&v:spur)margin=std::max(margin,4.-(q-v).norm());
  return J{{"route_distance",sqrt(distance)},{"progress",arc[best]},{"road_center_margin",margin}};
 }
};
namespace imac_ctrl {
struct LowerTrackingMpcTestAccess {using Impl=LowerTrackingMpcNode::Impl;static Impl&get(LowerTrackingMpcNode&n){return *n.impl_;}};
struct LowerPathGenerationTestAccess {
 using Node=SdMapUpperPlannerNode;
 struct Memory {Points previous;LowerPathGeometry path;Node::IntervalCenteringState interval;int wp=0;double last_success=-100;};
 static void setup(Node&n,const J&c){n.planner_timer_->cancel();g_soft_max_len=c.value("soft_cap",1e300);if(c.contains("soft_ratio"))n.preview_interval_soft_ratio_=c["soft_ratio"].get<double>();if(c.contains("lambda_relative_turn"))n.lambda_relative_turn_=c["lambda_relative_turn"].get<double>();if(c.contains("upper_r_dpsi"))n.upper_r_dpsi_=c["upper_r_dpsi"].get<double>();
  n.lambda_relative_turn_=c.value("lambda_relative_turn",n.lambda_relative_turn_);
  n.upper_r_dpsi_=c.value("upper_r_dpsi",n.upper_r_dpsi_);
  n.terminal_position_weight_multiplier_=c.value("terminal_position_weight_multiplier",n.terminal_position_weight_multiplier_);
  n.waypoint_switch_eps_m_=c.value("wp_switch_eps",n.waypoint_switch_eps_m_);
  n.preview_min_segment_samples_=c.value("preview_min_segment_samples",n.preview_min_segment_samples_);
 }
 static J plan(Node&n,Memory&m,const World&w,const V&position,double yaw,double speed,double t){
  Node::GridMapSnapshot g;g.valid=true;g.frame_id="Vehicle";g.width=g.height=128;g.resolution=.3125;g.origin_body=V(0,-20);g.data.resize(128*128);
  auto R=rotation(yaw);for(int y=0;y<128;++y)for(int x=0;x<128;++x)g.data[y*128+x]=w.cell(position+R*(g.origin_body+g.resolution*V(x+.5,y+.5)));
  auto body=[&](const V&q)->V{return R.transpose()*(q-position);};
  auto pair=egocentric_planner_geometry::findWaypointPair(w.route,m.wp,position,std::max(.01,speed)*V(cos(yaw),sin(yaw)),n.waypoint_switch_eps_m_,n.waypoint_switch_distance_m_);m.wp=pair[0];
  int horizon=n.computeRequestedPreviewSteps(&g,n.target_speed_mps_);
  horizon=egocentric_planner_geometry::retainMatlabPreviewHorizon(horizon,m.previous.size(),n.preview_restore_full_horizon_);
  Points prefix;bool used=m.previous.size()==static_cast<size_t>(horizon+1)&&egocentric_planner_geometry::buildMeasuredFrameProgressPrefix(m.previous,position,yaw,n.target_speed_mps_*n.upper_prediction_dt_sec_,horizon,prefix);
  if(!used)m.previous.clear();
  auto ref=n.buildPreviewReference(m.wp>0?body(w.route[m.wp-1]):V::Zero(),body(w.route[m.wp]),body(w.route[pair[1]]),m.wp>0,n.target_speed_mps_,horizon,prefix);
  auto c=n.extractPreviewIntervals(ref,&g,m.interval);m.interval=c.interval_update.state;
  n.applyTerminalPositionTargets(c,body(w.route[m.wp]),body(w.route[pair[1]]),n.target_speed_mps_*n.upper_prediction_dt_sec_);
  auto z=n.solveUpperMiqp(c,n.target_speed_mps_);
  J out={{"t",t},{"valid",z.valid},{"status",z.status},{"branch",z.branch_step},{"wp",m.wp},{"horizon",c.N_pred},{"counts",c.Nck},{"B",m.interval.reference_length},{"road_turn",c.delta_psi_road_rad},{"used_previous",used},{"solve_ms",z.solver_only_time_ms},{"turn_steps",z.turn_window_steps},{"turn_targets",z.turn_target_rad},{"turn_cost",z.turn_cost},{"cands",[&]{J a=J::array();for(size_t k=0;k<c.pmk.size();++k){J row=J::array();for(size_t i=0;i<c.pmk[k].size();++i){V p0=position+R*c.pmk[k][i],p1=position+R*c.pMk[k][i];row.push_back({p0.x(),p0.y(),p1.x(),p1.y()});}a.push_back(row);}return a;}()},{"ref",[&]{Points q;for(const auto&p:c.prk)q.push_back(position+R*p);return packed(q);}()},{"selected",z.selected_corridors},{"pos",{position.x(),position.y()}},{"yaw",yaw}};
  if(z.valid){Points world;for(const auto&q:z.predicted_body)world.push_back(position+R*q);std::vector<double>s;auto dense=n.densifyPath(world,n.target_speed_mps_,s);m.path=prepareLowerPath(dense,&s);m.previous=world.size()>=3?world:Points{};m.last_success=t;out["path"]=packed(world);if(world.size()>1)out["heading"]=atan2((world[1]-world[0]).y(),(world[1]-world[0]).x());}
  return out;
 }
};
}
static double referenceError(const imac_ctrl::LowerPathGeometry&p,const V&q){
 double best=1e30;for(size_t i=1;i<p.points.size();++i){V v=p.points[i]-p.points[i-1];double a=std::clamp((q-p.points[i-1]).dot(v)/std::max(v.squaredNorm(),1e-15),0.,1.);best=std::min(best,(q-p.points[i-1]-a*v).norm());}return best;
}
static imac_ctrl::LowerPathGeometry blendPath(const imac_ctrl::LowerPathGeometry&next,const imac_ctrl::LowerPathGeometry&old,double alpha){
 if(alpha>=1||!old.valid)return next;
 // Compare world geometry at matched spatial progress. Never blend raw indices.
 double best=1e30,start=0;for(size_t i=1;i<old.points.size();++i){V v=old.points[i]-old.points[i-1];double a=std::clamp((next.points[0]-old.points[i-1]).dot(v)/std::max(v.squaredNorm(),1e-15),0.,1.);double d=(next.points[0]-old.points[i-1]-a*v).squaredNorm();if(d<best){best=d;start=old.s[i-1]+a*(old.s[i]-old.s[i-1]);}}
 Points q=next.points;for(size_t i=1;i<q.size();++i){double query=start+next.s[i]-next.s[0];if(query>old.s.back())continue;auto it=std::upper_bound(old.s.begin(),old.s.end(),query);size_t k=std::clamp(static_cast<size_t>(it-old.s.begin()),size_t(1),old.s.size()-1);double a=(query-old.s[k-1])/(old.s[k]-old.s[k-1]);V previous=(1-a)*old.points[k-1]+a*old.points[k];q[i]=alpha*q[i]+(1-alpha)*previous;}
 return imac_ctrl::prepareLowerPath(q,&next.s);
}
static J configsFor(const J&input,const J&scene){J configs=input["configs"];if(scene.value("mode",std::string("live"))=="live")for(const auto&c:input["live_extra_configs"])configs.push_back(c);return configs;}
static J run(const World&w,const J&config,const J&plant,const std::string&file){
 rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",file,"--log-level","error"});
 opt.parameter_overrides({rclcpp::Parameter("mavlink_enable",false),rclcpp::Parameter("pixhawk_output_backend","disabled"),rclcpp::Parameter("publish_pwm",false)});
 auto upper=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);auto lower=std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);
 using UA=imac_ctrl::LowerPathGenerationTestAccess;using LA=imac_ctrl::LowerTrackingMpcTestAccess;
 UA::setup(*upper,config);auto&impl=LA::get(*lower);impl.controller_timer_->cancel();
 if(impl.mavlinkActive())throw std::runtime_error("hardware backend must be disabled");
 impl.lower_steering_actuator_time_constant_sec_=config.value("model_tau",impl.lower_steering_actuator_time_constant_sec_);
 impl.resetSteeringActuatorModelState();
 impl.lower_rd_steering_rate_=config.value("lower_rd_steering_rate",impl.lower_rd_steering_rate_);
 impl.lower_q_heading_=config.value("lower_q_heading",impl.lower_q_heading_);
 impl.lower_r_steering_=config.value("lower_r_steering",impl.lower_r_steering_);
 impl.lower_max_steering_rate_radps_=config.value("lower_max_steering_rate_radps",impl.lower_max_steering_rate_radps_);
 impl.lower_prediction_steps_=config.value("lower_prediction_steps",impl.lower_prediction_steps_);
 impl.enable_output_filter_=config.value("enable_output_filter",impl.enable_output_filter_);
 impl.steer_norm_rate_max_=config.value("steer_norm_rate_max",impl.steer_norm_rate_max_);
 impl.steer_norm_lpf_alpha_=config.value("steer_norm_lpf_alpha",impl.steer_norm_lpf_alpha_);
 UA::Memory memory;J plans=J::array(),states=J::array();V xy(w.scene["start"][0],w.scene["start"][1]);double yaw=w.scene["start"][2],delta=w.scene.value("initial_delta",0.);
 int delay=std::lround(plant["delay"].get<double>()/.001);std::deque<double> pending(delay,0.);double tau=plant["tau"];
 std::string outcome="time_limit";double offroad_time=0;double duration=w.scene["duration"];
 std::string mode=w.scene.value("mode",std::string("live"));
 size_t reference_index=0,speed_index=0;imac_ctrl::LowerPathGeometry original,filtered;
 if(mode=="fixed"){original=imac_ctrl::prepareLowerPath(w.center,&w.arc);memory.path=original;}

 for(int step=0;step<static_cast<int>(duration*10);++step){double t=step*.1,v=std::min(6.,w.scene.value("v0",0.)+1.5*t);
  if(mode=="replay"){const auto&profile=w.scene["speed_profile"];while(speed_index+1<profile.size()&&profile[speed_index+1][0].get<double>()<=t+.0001)++speed_index;v=profile[speed_index][1];}
  double measured_yaw=yaw+plant["yaw_noise_deg"].get<double>()*M_PI/180*sin(1.3*t);
  bool changed=false;
  if(mode=="live"&&step%10==0){auto result=UA::plan(*upper,memory,w,xy,measured_yaw,v,t);changed=result["valid"];plans.push_back(result);if(changed)original=memory.path;}
  if(mode=="replay"){
   const auto&refs=w.scene["references"];
   while(reference_index<refs.size()&&refs[reference_index]["t"].get<double>()<=t+.0001){const auto&r=refs[reference_index++];auto q=pts(r["xy"]);auto ss=r["s"].get<std::vector<double>>();original=imac_ctrl::prepareLowerPath(q,&ss);memory.last_success=t;changed=true;}
  }
  if(mode=="fixed")memory.last_success=t;
  if(changed||(!filtered.valid&&original.valid)){
   filtered=blendPath(original,filtered,config.value("path_alpha",1.));
   memory.path=filtered;
  }else if(mode=="live"&&filtered.valid){memory.path=filtered;}

  auto measurements=w.measure(xy);double progress=measurements["progress"];
  if(mode!="replay"&&progress>w.arc.back()-3&&(xy-w.center.back()).norm()<4){outcome="goal_reached";break;}
  bool stale=t-memory.last_success>1.5;if(t-memory.last_success>6.){outcome="stale_path_stop";break;}
  if(w.cell(xy)==100){offroad_time+=.1;}else{offroad_time=0;} if(offroad_time>1.0){outcome="offroad_stop";break;}
  LA::Impl::PoseSnapshot pose;pose.position=xy;pose.yaw_rad=measured_yaw;pose.speed_mps=v;pose.valid=pose.speed_valid=true;pose.frame_id="map";
  auto z=impl.solveTrackingMpc(pose,memory.path);
  double command=stale?0.:(z.valid?z.steering_sequence_rad.front():(z.fallback_valid?z.fallback_steering_rad:0.));
  double norm=impl.filterSteering(command,rclcpp::Time(static_cast<int64_t>((t+1.)*1e9),RCL_ROS_TIME));double applied=norm*impl.steeringCommandScale();
  J row={{"t",t},{"x",xy.x()},{"y",xy.y()},{"yaw",yaw},{"v",v},{"delta",delta},{"command",applied},{"lower_valid",z.valid&&!stale},{"stale",stale},{"cell",w.cell(xy)},{"fallback",!z.valid&&z.fallback_valid},{"ey",z.lateral_error_m},{"epsi",z.heading_error_rad},{"route_distance",measurements["route_distance"]},{"progress",progress},{"road_center_margin",measurements["road_center_margin"]}};row["original_ref_error"]=referenceError(original,xy);row["yaw_rate"]=v/2.8*cos(atan(.5*tan(delta)))*tan(delta);row["reference_index"]=reference_index;states.push_back(row);
  if(mode=="live"&&measurements["road_center_margin"].get<double>()< -2){outcome="left_road_stop";break;}
  for(int j=0;j<100;++j){double u=applied;if(delay){u=pending.front();pending.pop_front();pending.push_back(applied);}delta+=(1-exp(-.001/tau))*(u-delta);double beta=atan(.5*tan(delta));double rate=v/2.8*cos(beta)*tan(delta);xy+=.001*v*V(cos(yaw+beta+.0005*rate),sin(yaw+beta+.0005*rate));yaw=wrapToPi(yaw+.001*rate);}
  impl.advanceSteeringActuatorModel(applied);
 }
 return J{{"scene",w.scene["name"]},{"config",config},{"plant",plant},{"outcome",outcome},{"plans",plans},{"states",states},{"final_xy",{xy.x(),xy.y()}},{"length",w.arc.back()}};
}
int main(int argc,char**argv){if(argc!=3){std::cerr<<"sim INPUT_JSON OUTPUT_JSONL\n";return 2;}J input;std::ifstream(argv[1])>>input;rclcpp::init(0,nullptr);std::ofstream output(argv[2]);
 for(const auto&scene:input["scenes"]){World w(scene);for(const auto&plant:input["plants"])for(const auto&config:configsFor(input,scene)){auto r=run(w,config,plant,input["config_file"]);output<<r.dump()<<'\n';output.flush();std::cout<<scene["name"]<<" "<<plant["name"]<<" "<<config["name"]<<" "<<r["outcome"]<<" cycles="<<r["states"].size()<<std::endl;}}
 rclcpp::shutdown();return 0;
}
