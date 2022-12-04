// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  auto heightmap = world.addHeightMap(binaryPath.getDirectory() + "\\rsc\\raisimUnrealMaps\\mountain1.png",
                                      0, 0, 151.2, 151.2, 15.3/(39142-32640), -32640 * 15.3/(39142-32640));
  heightmap->setAppearance("hidden");
  auto aliengo = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");

  /// aliengo joint PD controller
  Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
  jointNominalConfig << 0, 0, 1.24, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
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
  server.setMap("mountain1");
  server.focusOn(aliengo);
  server.launchServer();

  for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
