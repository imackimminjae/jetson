// Standalone numerical harness. No spinning, command publication, or hardware I/O.
#define VIRTUAL_CONTROL_LOWER_MPC_NO_MAIN
#include "../src/virtual_control/src/lower_tracking_mpc_node.cpp"
#include "virtual_control/tracking_control.hpp"
#include "virtual_control/egocentric_planner_geometry.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
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
 static void setup(Node&n,const J&c){n.planner_timer_->cancel();
  n.lambda_relative_turn_=c.value("lambda_relative_turn",n.lambda_relative_turn_);
  n.upper_r_dpsi_=c.value("upper_r_dpsi",n.upper_r_dpsi_);
  n.terminal_position_weight_multiplier_=c.value("terminal_position_weight_multiplier",n.terminal_position_weight_multiplier_);
  n.waypoint_switch_eps_m_=c.value("wp_switch_eps",n.waypoint_switch_eps_m_);
  n.preview_min_segment_samples_=c.value("preview_min_segment_samples",n.preview_min_segment_samples_);
 }
 static J plan(Node&n,Memory&m,const World&w,const V&position,double yaw,double speed,double t){
  Node::GridMapSnapshot g;g.valid=true;g.frame_id="Vehicle";g.width=g.height=128;g.resolution=.3125;g.origin_body=V(2.95,-20);g.data.resize(128*128);
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
  J out={{"t",t},{"valid",z.valid},{"status",z.status},{"branch",z.branch_step},{"wp",m.wp},{"horizon",c.N_pred},{"counts",c.Nck},{"B",m.interval.reference_length},{"road_turn",c.delta_psi_road_rad},{"used_previous",used},{"solve_ms",z.solver_only_time_ms}};
  if(z.valid){Points world;for(const auto&q:z.predicted_body)world.push_back(position+R*q);std::vector<double>s;auto dense=n.densifyPath(world,n.target_speed_mps_,s);m.path=prepareLowerPath(dense,&s);m.previous=world.size()>=3?world:Points{};m.last_success=t;out["path"]=packed(world);if(world.size()>1)out["heading"]=atan2((world[1]-world[0]).y(),(world[1]-world[0]).x());}
  return out;
 }
};
}
static J run(const World&w,const J&config,const J&plant,const std::string&file){
 rclcpp::NodeOptions opt;opt.arguments({"--ros-args","--params-file",file,"--log-level","error"});
 opt.parameter_overrides({rclcpp::Parameter("mavlink_enable",false),rclcpp::Parameter("pixhawk_output_backend","disabled"),rclcpp::Parameter("publish_pwm",false)});
 auto upper=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(opt);auto lower=std::make_shared<imac_ctrl::LowerTrackingMpcNode>(opt);
 using UA=imac_ctrl::LowerPathGenerationTestAccess;using LA=imac_ctrl::LowerTrackingMpcTestAccess;
 UA::setup(*upper,config);auto&impl=LA::get(*lower);impl.controller_timer_->cancel();
 if(impl.mavlinkActive())throw std::runtime_error("hardware backend must be disabled");
 impl.lower_rd_steering_rate_=config.value("lower_rd_steering_rate",impl.lower_rd_steering_rate_);
 impl.enable_output_filter_=config.value("enable_output_filter",impl.enable_output_filter_);
 impl.steer_norm_rate_max_=config.value("steer_norm_rate_max",impl.steer_norm_rate_max_);
 impl.steer_norm_lpf_alpha_=config.value("steer_norm_lpf_alpha",impl.steer_norm_lpf_alpha_);
 UA::Memory memory;J plans=J::array(),states=J::array();V xy(w.scene["start"][0],w.scene["start"][1]);double yaw=w.scene["start"][2],delta=0;
 int delay=std::lround(plant["delay"].get<double>()/.001);std::deque<double> pending(delay,0.);double tau=plant["tau"];
 std::string outcome="time_limit";double duration=w.scene["duration"];
 for(int step=0;step<static_cast<int>(duration*10);++step){double t=step*.1,v=std::min(6.,1.5*t);double measured_yaw=yaw+plant["yaw_noise_deg"].get<double>()*M_PI/180*sin(1.3*t);
  if(step%2==0)plans.push_back(UA::plan(*upper,memory,w,xy,measured_yaw,v,t));
  auto measurements=w.measure(xy);double progress=measurements["progress"];
  if(progress>w.arc.back()-3&&(xy-w.center.back()).norm()<4){outcome="goal_reached";break;}
  if(t-memory.last_success>1.5){outcome="stale_path_stop";break;}
  LA::Impl::PoseSnapshot pose;pose.position=xy;pose.yaw_rad=measured_yaw;pose.speed_mps=v;pose.valid=pose.speed_valid=true;pose.frame_id="map";
  auto z=impl.solveTrackingMpc(pose,memory.path);
  double command=z.valid?z.steering_sequence_rad.front():(z.fallback_valid?z.fallback_steering_rad:0.);
  double norm=impl.filterSteering(command,rclcpp::Time(static_cast<int64_t>((t+1.)*1e9),RCL_ROS_TIME));double applied=norm*impl.steeringCommandScale();
  J row={{"t",t},{"x",xy.x()},{"y",xy.y()},{"yaw",yaw},{"v",v},{"delta",delta},{"command",applied},{"lower_valid",z.valid},{"fallback",!z.valid&&z.fallback_valid},{"ey",z.lateral_error_m},{"epsi",z.heading_error_rad},{"route_distance",measurements["route_distance"]},{"progress",progress},{"road_center_margin",measurements["road_center_margin"]}};states.push_back(row);
  if(measurements["road_center_margin"].get<double>()< -2){outcome="left_road_stop";break;}
  for(int j=0;j<100;++j){double u=applied;if(delay){u=pending.front();pending.pop_front();pending.push_back(applied);}delta+=(1-exp(-.001/tau))*(u-delta);double beta=atan(.5*tan(delta));double rate=v/2.8*cos(beta)*tan(delta);xy+=.001*v*V(cos(yaw+beta+.0005*rate),sin(yaw+beta+.0005*rate));yaw=wrapToPi(yaw+.001*rate);}
  impl.advanceSteeringActuatorModel(applied);
 }
 return J{{"scene",w.scene["name"]},{"config",config},{"plant",plant},{"outcome",outcome},{"plans",plans},{"states",states},{"final_xy",{xy.x(),xy.y()}},{"length",w.arc.back()}};
}
int main(int argc,char**argv){if(argc!=3){std::cerr<<"sim INPUT_JSON OUTPUT_JSONL\n";return 2;}J input;std::ifstream(argv[1])>>input;rclcpp::init(0,nullptr);std::ofstream output(argv[2]);
 for(const auto&scene:input["scenes"]){World w(scene);for(const auto&plant:input["plants"])for(const auto&config:input["configs"]){auto r=run(w,config,plant,input["config_file"]);output<<r.dump()<<'\n';output.flush();std::cout<<scene["name"]<<" "<<plant["name"]<<" "<<config["name"]<<" "<<r["outcome"]<<" cycles="<<r["states"].size()<<std::endl;}}
 rclcpp::shutdown();return 0;
}
