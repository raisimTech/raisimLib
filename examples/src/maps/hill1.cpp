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
  auto heightmap = world.addHeightMap(binaryPath.getDirectory() + "\\rsc\\raisimUnrealMaps\\hill1.png",
                                      0, 0, 504, 504, 38.0/(37312-32482), -32650 * 38.0/(37312-32482));
  heightmap->setAppearance("hidden");
  auto aliengo = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");

  /// aliengo joint PD controller
  Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
  jointNominalConfig << 0, 0, 10.24, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(aliengo->getDOF()), jointDgain(aliengo->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  aliengo->setGeneralizedCoordinate(jointNominalConfig);
  aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
  aliengo->setPdGains(jointPgain, jointDgain);
  aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
  aliengo->setName("aliengo");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.setMap("hill1");
  server.focusOn(aliengo);
  server.launchServer();

  for (int i=0; i<2000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    server.integrateWorldThreadSafe();
  }

  std::cout<<"mass "<<aliengo->getMassMatrix()[0]<<std::endl;

  server.killServer();
}
