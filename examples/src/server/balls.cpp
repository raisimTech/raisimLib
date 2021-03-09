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
  double dt = 0.003;

  raisim::World world;
  world.setTimeStep(dt);
  world.setERP(world.getTimeStep(), world.getTimeStep());

  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::ArticulatedSystem*> anymals;

  /// ANYmal joint PD controller
  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18),
      jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.tail(12).setConstant(200.0);
  jointDgain.tail(12).setConstant(10.0);

  const size_t N = 1;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      anymals.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf"));
      anymals.back()->setGeneralizedCoordinate(
          {double(2 * i), double(j), 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
           -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
      anymals.back()->setGeneralizedForce(
          Eigen::VectorXd::Zero(anymals.back()->getDOF()));
      anymals.back()->setControlMode(
          raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      anymals.back()->setPdGains(jointPgain, jointDgain);
      anymals.back()->setName("anymal" + std::to_string(j + i * N));
    }
  }

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.2);
  std::srand(std::time(nullptr));
  anymals.back()->printOutBodyNamesInOrder();

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  /// throw balls
  int interval = 600;
  int numBalls = 20;
  int j = 0;

  for (int i = 0;; i++) {
    raisim::MSLEEP(dt * 1000);

    if (i % interval == 0 && j < numBalls) {
      auto* ball = world.addSphere(0.1, 1.0);
      ball->setPosition(0, -2, 0.8);
      ball->setVelocity(0, 10, 0, 0, 0, 0);
      ball->setAppearance("red");
      j++;
    }

    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
