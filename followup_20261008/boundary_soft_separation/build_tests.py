from pathlib import Path
import shlex,subprocess
H=Path(__file__).resolve().parent;R=H.parents[1];B=R/'build/virtual_control'
s=(R/'src/virtual_control/test/test_lower_path_generation.cpp').read_text()
needle='  static void cancel(SdMapUpperPlannerNode & node)'
method='''  static auto edgeInsetScene(SdMapUpperPlannerNode & node, int side, double angle, int variant) {
    SdMapUpperPlannerNode::GridMapSnapshot grid;
    grid.valid=true;grid.width=grid.height=128;grid.resolution=.3125;grid.origin_yaw_rad=angle;
    Eigen::Matrix2d R;R<<cos(angle),-sin(angle),sin(angle),cos(angle);
    grid.origin_body=R*Eigen::Vector2d(0,-20);grid.data.assign(128*128,100);
    for(int y=0;y<128;++y)for(int x=0;x<128;++x){
      const double yy=side*(-20+(y+.5)*.3125);
      if(yy>10)grid.data[y*128+x]=0;
      if(variant==2 && yy>19)grid.data[y*128+x]=-1;
      if(variant==3 && yy>19)grid.data[y*128+x]=100;
    }
    SdMapUpperPlannerNode::PreviewReferenceData ref;ref.valid=true;ref.used_previous_solution=true;
    const double yy=variant==1?14:variant==2||variant==3?18:variant==4?9:19;
    ref.points_body={Eigen::Vector2d::Zero(),R*Eigen::Vector2d(8,side*yy),R*Eigen::Vector2d(12,side*yy)};
    ref.heading_rad={angle,angle,angle};
    auto p=node.extractPreviewIntervals(ref,&grid,SdMapUpperPlannerNode::IntervalCenteringState{10});
    // Transform endpoints back to map axes for a symmetric left/right assertion.
    for(size_t k=0;k<p.pmk.size();++k)for(size_t i=0;i<p.pmk[k].size();++i){p.pmk[k][i]=R.transpose()*p.pmk[k][i];p.pMk[k][i]=R.transpose()*p.pMk[k][i];}
    return p;
  }
'''
assert s.count(needle)==1;s=s.replace(needle,method+needle)
s+='''
TEST(LowerPathGeneration, CandidateOnlyRelaxesReferenceExcludingLateralMapEdge) {
  rclcpp::init(0,nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("grid_positive_is_drivable",false),rclcpp::Parameter("grid_value_threshold",0),rclcpp::Parameter("preview_interval_soft_ratio",.7)});
  auto node=std::make_shared<imac_ctrl::SdMapUpperPlannerNode>(options);
  using A=imac_ctrl::LowerPathGenerationTestAccess;A::cancel(*node);
  for(int side:{-1,1})for(double angle:{0.,.7})for(int variant:{0,1,2,3,4}){
    auto p=A::edgeInsetScene(*node,side,angle,variant);
    ASSERT_EQ(p.N_pred,2);ASSERT_EQ(p.Nck[1],1);
    const auto& obs=p.interval_info[1][0].observation;
    const double W=obs.length,processed=p.processed_lengths[1][0];
    ASSERT_TRUE(p.interval_info[1][0].treatment.centering_on);
    if(variant==0){
      EXPECT_NEAR(processed,.7*W,1e-8);
      EXPECT_NEAR(std::max(side*p.pmk[1][0].y(),side*p.pMk[1][0].y()),20.,.10001);
      EXPECT_NEAR(p.interval_update.state.reference_length,10.,1e-8);
    }else{
      EXPECT_NEAR(processed,.4*W,1e-8); // Inside/unknown/observed/nonmember controls retain symmetric treatment.
    }
  }
  node.reset();rclcpp::shutdown();
}
'''
p=H/'candidate_tests.cpp';p.write_text(s);flags=[]
for l in (B/'CMakeFiles/test_lower_path_generation.dir/flags.make').read_text().splitlines():
 if l.startswith(('CXX_DEFINES =','CXX_INCLUDES =','CXX_FLAGS =')):flags+=shlex.split(l.split('=',1)[1])
flags=[f for f in flags if f!='-O3']+['-O1'];link=shlex.split((B/'CMakeFiles/test_lower_path_generation.dir/link.txt').read_text());libs=link[link.index('test_lower_path_generation')+1:];libs=[str(B/v) if v.startswith('gtest/') else v for v in libs]
with (H/'tests_build.log').open('w') as log:
 subprocess.run(['/usr/bin/c++',*flags,'-c',str(p),'-o',str(H/'tests.o')],check=True,stdout=log,stderr=subprocess.STDOUT)
 subprocess.run(['/usr/bin/c++',str(H/'tests.o'),str(H/'candidate.o'),'-o',str(H/'candidate_tests'),*libs],check=True,stdout=log,stderr=subprocess.STDOUT)
print('Built isolated tests')
