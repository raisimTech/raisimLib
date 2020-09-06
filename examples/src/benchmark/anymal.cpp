// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <chrono>

#include "benchmarkCommon.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  /// get binary path
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  raisim::World sim;
  sim.setTimeStep(0.002);
  sim.setERP(0., 0.);

  auto checkerBoard = sim.addGround();

  Eigen::VectorXd jointConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointVel(18), jointPgain(18), jointDgain(18);

  jointPgain.setZero();
  jointPgain.tail(12).setConstant(200.0);

  jointDgain.setZero();
  jointDgain.tail(12).setConstant(10.0);

  jointVelocityTarget.setZero();

  jointConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVel << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  auto anymal = sim.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  const int loopN = 1000000;
  std::chrono::steady_clock::time_point begin, end;

  begin = std::chrono::steady_clock::now();
  for (int i = 0; i < loopN; i++) sim.integrate();

  end = std::chrono::steady_clock::now();

  raisim::print_timediff("anymal", loopN, begin, end);

  std::cout << "Press Enter to Continue";
  std::cin.ignore();
}
