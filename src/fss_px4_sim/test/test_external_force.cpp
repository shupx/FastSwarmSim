#include <gtest/gtest.h>
#include "fss_px4_sim/quadrotor_dynamics.hpp"

using MavrosQuadSimulator::Dynamics;

TEST(ExternalForce, WorldFrameMassScalingAndClear)
{
  Dynamics dynamics;
  dynamics.setMass(2.0);
  dynamics.setPos(0.0, 0.0, 10.0);
  // Rotate the aircraft: external force must still point east in world ENU.
  dynamics.setRPY(0.0, 0.0, 1.5707963267948966);
  Dynamics::Input input;
  input.omega = Eigen::Vector3d::Zero();
  input.thrust = 2.0 * dynamics.getGravityAcc();
  dynamics.setInput(input);
  dynamics.setExternalForce(Eigen::Vector3d(4.0, 0.0, -2.0));
  dynamics.step(0.0, 0.1);
  const auto state = dynamics.getState();
  EXPECT_NEAR(state.vel.x(), 0.2, 1e-8);
  EXPECT_NEAR(state.vel.y(), 0.0, 1e-8);
  EXPECT_NEAR(state.vel.z(), -0.1, 1e-8);
  EXPECT_NEAR(state.pos.x(), 0.01, 1e-8);
  dynamics.setExternalForce(Eigen::Vector3d::Zero());
  dynamics.step(0.1, 0.2);
  EXPECT_NEAR(dynamics.getState().vel.x(), state.vel.x(), 1e-8);
  EXPECT_NEAR(dynamics.getState().vel.z(), state.vel.z(), 1e-8);
}
