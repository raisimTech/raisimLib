// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();
  auto cartPole = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\cartPole\\cartpole.urdf");

  /// cartPole state
  Eigen::VectorXd jointNominalConfig(cartPole->getGeneralizedCoordinateDim());
  jointNominalConfig.setZero();
  jointNominalConfig[1] = 0.01;
  cartPole->setGeneralizedCoordinate(jointNominalConfig);
  
  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(cartPole);

  for (int i=0; i<200000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
