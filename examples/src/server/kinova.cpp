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
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();
  auto kinova = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\kinova\\urdf\\kinova.urdf");

  /// kinova joint PD controller
  Eigen::VectorXd jointNominalConfig(kinova->getGeneralizedCoordinateDim()), jointVelocityTarget(kinova->getDOF());
  jointNominalConfig << 0.0, 2.76, -1.57, 0.0, 2.0, 0.0;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(kinova->getDOF()), jointDgain(kinova->getDOF());
  jointPgain << 40.0, 40.0, 40.0, 15.0, 15.0, 15.0;
  jointDgain << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;

  kinova->setGeneralizedCoordinate(jointNominalConfig);
  kinova->setGeneralizedForce(Eigen::VectorXd::Zero(kinova->getDOF()));
  kinova->setPdGains(jointPgain, jointDgain);
  kinova->setPdTarget(jointNominalConfig, jointVelocityTarget);
  kinova->setName("kinova");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(kinova);

  for (int i=0; i<2000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    server.integrateWorldThreadSafe();
  }

  std::cout<<"mass "<<kinova->getMassMatrix()[0]<<std::endl;

  server.killServer();
}
