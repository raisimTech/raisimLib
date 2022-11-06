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

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.0001);

  /// create objects
  world.addGround();
  auto revAndPrisSpringAndDamper = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\springDamper\\cartpole.urdf");
  auto ballSpringAndDamper = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\springDamper\\chainSpringed.urdf");

  revAndPrisSpringAndDamper->setName("rev_pris_joint");
  ballSpringAndDamper->setName("ball_joint");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(ballSpringAndDamper);

  for (int i=0; i<2000000; i++) {
    raisim::MSLEEP(1);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
