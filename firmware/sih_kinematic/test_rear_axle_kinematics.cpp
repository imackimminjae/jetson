#include "rear_axle_kinematics.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace sih_kinematic;
void near(double a, double b, double eps = 1e-9) { assert(std::abs(a-b) < eps); }

int main()
{
  // Straight acceleration: integral of v(t)=t is 0.5*t^2, at an arbitrary yaw.
  RearAxleState s; MotionSample m;
  s.yaw_rad = 0.7;
  for (int i=1; i<=600; ++i) {
    assert(advanceRearAxle(s,m,i*.01,0.0,2.8,.01));
  }
  near(s.x_m,18*std::cos(.7)); near(s.y_m,18*std::sin(.7));
  near(s.forward_speed_mps,6); near(m.forward_acceleration_mps2,1);

  // Independent analytic circle oracle. Include both turns and reverse motion.
  for (double steer : {-0.3, 0.3}) {
    for (double speed : {-3.0, 6.0}) {
      const double radius=2.8/std::tan(steer);
      const double period=2*kPi*std::abs(radius/speed);
      s=RearAxleState{}; s.forward_speed_mps=speed;
      for (int i=0; i<1000; ++i) {
        assert(advanceRearAxle(s,m,speed,steer,2.8,period/1000));
        near(std::hypot(s.x_m,s.y_m-radius),std::abs(radius),1e-8);
        near(-std::sin(s.yaw_rad)*m.vx_mps+std::cos(s.yaw_rad)*m.vy_mps,0);
        near(std::cos(s.yaw_rad)*m.vx_mps+std::sin(s.yaw_rad)*m.vy_mps,speed);
        near(m.lateral_acceleration_mps2,speed*speed/radius);
      }
      near(s.x_m,0,1e-8); near(s.y_m,0,1e-8); near(s.yaw_rad,0,1e-8);
    }
  }

  // At rest, steering must not rotate the chassis or create lateral motion.
  s=RearAxleState{};
  for(int i=0; i<100; ++i) {assert(advanceRearAxle(s,m,0,.35,2.8,.01));}
  near(s.x_m,0); near(s.y_m,0); near(s.yaw_rad,0); near(m.yaw_rate_radps,0);

  // A speed ramp followed by equal braking comes to rest after 6 m.
  s=RearAxleState{};
  for(int i=1;i<=300;++i) {assert(advanceRearAxle(s,m,i*.01,0,2.8,.01));}
  for(int i=1;i<=100;++i) {assert(advanceRearAxle(s,m,3-i*.03,0,2.8,.01));}
  near(s.x_m,6); near(s.forward_speed_mps,0); near(m.vx_mps,0);

  // A near-zero angle must remain finite and agree with the straight limit.
  s=RearAxleState{}; s.forward_speed_mps=6;
  assert(advanceRearAxle(s,m,6,1e-12,2.8,.1)); near(s.x_m,.6); near(s.y_m,0);

  // Rejected data must never partially overwrite the state or sensor sample.
  const auto previous=s; const auto previous_sample=m;
  assert(!advanceRearAxle(s,m,6,.3,0,.01));
  assert(!advanceRearAxle(s,m,6,.3,2.8,0));
  assert(!advanceRearAxle(s,m,6,kPi/2,2.8,.01));
  assert(!advanceRearAxle(s,m,std::numeric_limits<double>::quiet_NaN(),.3,2.8,.01));
  near(s.x_m,previous.x_m); near(s.yaw_rad,previous.yaw_rad);
  near(m.vx_mps,previous_sample.vx_mps);
  std::cout << "PASS: acceleration, signed circular motion, zero slip, standstill, braking, small angle, invalid inputs\n";
}
