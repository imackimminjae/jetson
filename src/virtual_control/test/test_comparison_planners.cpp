#include "virtual_control/comparison_planner.hpp"
#include "virtual_control/lower_path_geometry.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace imac_ctrl::comparison;  // NOLINT
namespace
{
Input fixture()
{
  Input in;
  in.grid.width=160; in.grid.height=80; in.grid.resolution=0.25; in.grid.origin={-5,-10};
  in.grid.data.assign(in.grid.width*in.grid.height,0);
  in.geometry={0.3,0.42,0.2,0.15,0.3,1.0};
  in.settings.extra_margin=0.1; in.settings.prediction_speed=2;
  in.settings.steps=5; in.settings.ring_step=1;
  in.route={{0,0},{10,0},{25,0}}; in.actual_speed=2; in.applied_steer=0;
  in.progress=0; in.time=1;
  return in;
}
void fillRectangle(Input & in,double x0,double x1,double y0,double y1,int8_t value)
{
  for (int y=0; y<in.grid.height; ++y) {
    for (int x=0; x<in.grid.width; ++x) {
      const auto p=in.grid.cellCenter(x,y);
      if (p.x()>=x0 && p.x()<=x1 && p.y()>=y0 && p.y()<=y1) {
        in.grid.data[y*in.grid.width+x]=value;
      }
    }
  }
}
}  // namespace
TEST(ComparisonGrid, RotationPolarityUnknownAndFloor)
{
  auto in=fixture(); auto & g=in.grid;
  g.origin={1,-2}; g.origin_yaw=0.6; g.frame_world={{4,5},-0.3};
  const auto p=g.cellCenter(3,4);
  EXPECT_NEAR(g.toGrid(p).x(),3.5,1e-10); EXPECT_NEAR(g.toGrid(p).y(),4.5,1e-10);
  EXPECT_EQ(g.cell(p),Cell::Free);
  g.data[4*g.width+3]=100; EXPECT_EQ(g.cell(p),Cell::Occupied);
  g.positive_is_drivable=true; EXPECT_EQ(g.cell(p),Cell::Free);
  g.data[4*g.width+3]=-1; EXPECT_EQ(g.cell(p),Cell::Unknown);
  const auto outside=g.frame_world.p+rotate(g.origin+rotate(Point(-0.01,0.1),g.origin_yaw),g.frame_world.yaw);
  EXPECT_EQ(g.cell(outside),Cell::Outside);
}
TEST(ComparisonGeometry, CorridorNeverBridgesDisconnectedBranches)
{
  auto in=fixture();
  fillRectangle(in,-5,35,1.5,3.5,100);
  const auto c=corridor(in,{{5,0},{1,0},0}); ASSERT_TRUE(c);
  EXPECT_LT((*c)[1],1.5); EXPECT_GT((*c)[0],-10);
  EXPECT_FALSE(corridor(in,{{5,2.5},{1,0},0}));
}
TEST(ComparisonGeometry, FreeSpaceIndexPreservesRotatedFootprintDecisions)
{
  auto raw=fixture(); fillRectangle(raw,2,4,-2,2,100);
  fillRectangle(raw,7,9,1,3,-1);
  auto indexed=raw; indexed.grid.buildIndex();
  for (int i=0; i<40; ++i) {
    const Pose pose{{0.2*i,0.15*(i%5)-0.3},0.12*(i%7)};
    EXPECT_EQ(footprintFree(raw,pose),footprintFree(indexed,pose));
  }
}
TEST(ComparisonGeometry, ConnectorCrossesObstacleEvenWithFreeEndpoints)
{
  auto in=fixture();
  fillRectangle(in,2.2,2.8,-2,2,100);
  EXPECT_TRUE(footprintFree(in,{{1,0},0})); EXPECT_TRUE(footprintFree(in,{{4,0},0}));
  EXPECT_FALSE(connectionFree(in,{{1,0},0},{{4,0},0}));
  NvpPlanner nvp; auto out=nvp.plan(in);
  EXPECT_FALSE(out.success); EXPECT_FALSE(out.reason.empty()); EXPECT_TRUE(out.path.empty());
}
TEST(ComparisonGeometry, UnknownPolicyOnlyExemptsInitialFootprint)
{
  auto in=fixture(); fillRectangle(in,-5,-0.1,-10,10,-1);
  EXPECT_TRUE(footprintFree(in,in.vehicle));
  in.settings.allow_initial_unobserved_footprint=false;
  EXPECT_FALSE(footprintFree(in,in.vehicle));
  in.settings.allow_initial_unobserved_footprint=true;
  fillRectangle(in,2,3,-10,10,-1);
  EXPECT_FALSE(connectionFree(in,{{1,0},0},{{4,0},0}));
  fillRectangle(in,-1,1,-1,1,100); EXPECT_FALSE(footprintFree(in,in.vehicle));
}
TEST(ComparisonNvp, RejectsBoundaryCrossingBetweenValleyRings)
{
  auto in=fixture(); in.settings.prediction_speed=2; in.settings.steps=10;
  in.settings.ring_step=3; in.settings.max_connection=4;
  fillRectangle(in,4.2,4.8,-10,10,100);
  // Both ring centers are free; a thin cross-road divider lies between them.
  ASSERT_TRUE(footprintFree(in,{{3,0},0})); ASSERT_TRUE(footprintFree(in,{{6,0},0}));
  NvpPlanner nvp; const auto out=nvp.plan(in);
  EXPECT_FALSE(out.success); EXPECT_GT(out.candidates,0);
  EXPECT_NE(out.reason.find("connector"),std::string::npos)<<out.reason;
}
TEST(ComparisonCurve, TangentCurvatureProjectionAndOpenEnd)
{
  ReferenceCurve line({{0,0},{10,0},{20,0}});
  EXPECT_NEAR(line.length(),20,1e-6); EXPECT_NEAR(line.at(30).p.x(),20,1e-6);
  EXPECT_NEAR(line.at(5).tangent.x(),1,1e-6); EXPECT_NEAR(line.at(5).curvature,0,1e-6);
  EXPECT_NEAR(line.project({9,3},0,20),9,0.01);
  EXPECT_NEAR(line.project({19,0},0,3),3,0.01);
  ReferenceCurve turn({{0,0},{5,0},{7,2},{7,7}});
  EXPECT_GT(std::abs(turn.at(5).curvature),0.01);
}
TEST(ComparisonNvp, ForwardPathAndAtomicArc)
{
  auto in=fixture(); NvpPlanner nvp; auto out=nvp.plan(in);
  ASSERT_TRUE(out.success)<<out.reason;
  EXPECT_EQ(out.path.size(),out.s.size()); EXPECT_NEAR(out.path.front().x(),0,1e-8);
  EXPECT_GT(out.path.back().x(),4); EXPECT_GT(out.candidates,0);
  EXPECT_TRUE(imac_ctrl::prepareLowerPath(out.path,&out.s).valid);
  for (size_t i=1; i<out.path.size(); ++i) {EXPECT_GT(out.path[i].x(),out.path[i-1].x());}
  EXPECT_GE(out.total_ms,out.planning_ms);
}
TEST(ComparisonNvp, ForwardOnlyFullscaleBev)
{
  auto in=fixture(); in.grid.origin={0,-10}; in.geometry=Vehicle{};
  in.settings.extra_margin=1.1; in.settings.prediction_speed=6; in.settings.steps=7;
  in.route={{0,0},{50,0}};
  NvpPlanner nvp; auto out=nvp.plan(in); EXPECT_TRUE(out.success)<<out.reason;
}
TEST(ComparisonMpcc, ActualProgressStateAndSteeringBounds)
{
  auto in=fixture(); MpccPlanner mpcc;
  in.applied_steer=0.12;
  const auto out=mpcc.plan(in); ASSERT_TRUE(out.success)<<out.reason;
  ASSERT_EQ(out.progress.size(),out.steering.size()+1);
  EXPECT_EQ(out.qp_status,1); EXPECT_LT(out.max_constraint_violation,1e-4);
  double prior=*in.applied_steer;
  for (size_t k=0; k<out.steering.size(); ++k) {
    EXPECT_LE(std::abs(out.steering[k]),in.geometry.max_steer+1e-5);
    EXPECT_LE(std::abs(out.steering[k]-prior),in.geometry.max_steer_rate*
      (k==0 ? in.settings.period : in.settings.dt)+1e-5);
    EXPECT_GE(out.virtual_speed[k],-1e-5);
    EXPECT_LE(out.virtual_speed[k],in.settings.max_virtual_speed+1e-5);
    EXPECT_NEAR(out.progress[k+1]-out.progress[k],in.settings.dt*out.virtual_speed[k],1e-5);
    prior=out.steering[k];
  }
  EXPECT_TRUE(imac_ctrl::prepareLowerPath(out.path,&out.s).valid);
}
TEST(ComparisonMpcc, FullscaleForwardBev)
{
  auto in=fixture(); in.geometry=Vehicle{};
  in.grid.width=128; in.grid.height=128; in.grid.resolution=0.3125;
  in.grid.origin={0,-20}; in.grid.data.assign(128*128,0);
  in.settings=Settings{}; in.actual_speed=6; in.route={{0,0},{50,0}};
  MpccPlanner mpcc; const auto out=mpcc.plan(in); ASSERT_TRUE(out.success)<<out.reason;
  EXPECT_NEAR(out.path.back().x(),21,0.1);
  RecordProperty("total_ms",out.total_ms);
}
TEST(ComparisonMpcc, WarmStartUsesElapsedTimeAndReset)
{
  auto in=fixture(); MpccPlanner mpcc;
  ASSERT_TRUE(mpcc.plan(in).success);
  in.time+=0.2; in.vehicle.p.x()+=0.4;
  auto next=mpcc.plan(in); ASSERT_TRUE(next.success)<<next.reason;
  EXPECT_NEAR(next.warm_shift_sec,0.2,1e-8);
  mpcc.reset(); next=mpcc.plan(in); EXPECT_EQ(next.warm_shift_sec,0);
  in.route={{0,0},{20,0.1}}; next=mpcc.plan(in); EXPECT_EQ(next.warm_shift_sec,0);
}
TEST(ComparisonMpcc, ProjectionDoesNotJumpAtNearbyRepeatedSegments)
{
  auto in=fixture();
  in.route={{0,0},{20,0},{20,6},{0,6},{0,0.2},{20,0.2}};
  MpccPlanner mpcc; auto first=mpcc.plan(in); ASSERT_TRUE(first.success)<<first.reason;
  in.time+=0.2; in.vehicle.p={0.4,0.19};
  const auto out=mpcc.plan(in); EXPECT_LT(out.projected_s,3);
}
TEST(ComparisonMpcc, OpenEndShortensHorizonAndStops)
{
  auto in=fixture(); in.route={{0,0},{3.3,0}}; MpccPlanner mpcc;
  auto out=mpcc.plan(in); ASSERT_TRUE(out.success)<<out.reason;
  EXPECT_LE(out.path.back().x(),3.4); EXPECT_LE(out.progress.back(),3.3+1e-5);
  mpcc.reset(); in.vehicle.p={3.2,0}; in.progress=3.2;
  out=mpcc.plan(in); EXPECT_TRUE(out.goal_reached); EXPECT_FALSE(out.success);
  EXPECT_TRUE(out.path.empty()); EXPECT_FALSE(out.reason.empty());
  mpcc.reset(); in.vehicle.p={2.6,0}; in.progress=2.6;
  out=mpcc.plan(in); EXPECT_TRUE(out.success)<<out.reason;
  EXPECT_LE(out.path.back().x(),3.4);
}
TEST(ComparisonNvp, ShortTerminalApproachOutsideGoalRadius)
{
  auto in=fixture(); in.route={{0,0},{0.8,0}};
  NvpPlanner nvp; const auto out=nvp.plan(in); EXPECT_TRUE(out.success)<<out.reason;
  EXPECT_FALSE(out.goal_reached); EXPECT_NEAR(out.path.back().x(),0.8,0.05);
}
TEST(ComparisonMpcc, CurvedReferenceAndDistinctVirtualSpeed)
{
  auto in=fixture();
  in.route.clear();
  for (int i=0; i<=12; ++i) {
    const double angle=0.04*i;
    in.route.push_back({30*std::sin(angle),30*(1-std::cos(angle))});
  }
  in.actual_speed=0.0; // A stopped vehicle still receives nominal-speed geometric preview.
  MpccPlanner mpcc; const auto out=mpcc.plan(in); ASSERT_TRUE(out.success)<<out.reason;
  EXPECT_GT(out.path.back().y(),0.02);
  EXPECT_GT(out.virtual_speed.front(),0.5);
  EXPECT_EQ(in.actual_speed,0.0);
}
TEST(ComparisonMpcc, CollisionBetweenPredictionKnotsIsRejected)
{
  auto in=fixture();
  fillRectangle(in,0.35,0.65,-10,10,100);
  MpccPlanner mpcc; const auto out=mpcc.plan(in);
  EXPECT_FALSE(out.success); EXPECT_FALSE(out.reason.empty()); EXPECT_TRUE(out.path.empty());
}
TEST(ComparisonFailure, InvalidEmptyBlockedAndShort)
{
  auto in=fixture(); NvpPlanner nvp; MpccPlanner mpcc;
  in.grid.data.clear(); EXPECT_FALSE(nvp.plan(in).success); EXPECT_FALSE(mpcc.plan(in).success);
  in=fixture(); in.route.clear(); EXPECT_FALSE(nvp.plan(in).success); EXPECT_FALSE(mpcc.plan(in).success);
  in=fixture(); std::fill(in.grid.data.begin(),in.grid.data.end(),100);
  EXPECT_FALSE(nvp.plan(in).success); EXPECT_FALSE(mpcc.plan(in).success);
  in=fixture(); in.settings.steps=1; in.settings.dt=0.01;
  EXPECT_FALSE(nvp.plan(in).success); EXPECT_FALSE(mpcc.plan(in).success);
  std::vector<Point> dense; std::vector<double> s;
  EXPECT_FALSE(densify({{0,0},{0,0}},0.1,dense,s));
  EXPECT_FALSE(imac_ctrl::prepareLowerPath({{0,0},{1,0}},&s).valid);
}
