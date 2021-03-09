// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.003);

  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::ArticulatedSystem*> laikagos;

  /// laikago joint PD controller
  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18),
      jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointNominalConfig << 0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1,
      0.00, 0.5, -1, 0, 0.5, -0.7;

  jointPgain.tail(12).setConstant(200.0);
  jointDgain.tail(12).setConstant(10.0);

  const size_t N = 4;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      laikagos.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\laikago\\laikago.urdf"));
      laikagos.back()->setGeneralizedCoordinate(
          {double(2 * i), double(j), 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0,
           0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7});
      laikagos.back()->setGeneralizedForce(
          Eigen::VectorXd::Zero(laikagos.back()->getDOF()));
      laikagos.back()->setControlMode(
          raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      laikagos.back()->setPdGains(jointPgain, jointDgain);
      laikagos.back()->setPdTarget(jointNominalConfig, jointVelocityTarget);
      laikagos.back()->setName("laikago" + std::to_string(j + i * N));
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i = 0; i < 10000000; i++) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
