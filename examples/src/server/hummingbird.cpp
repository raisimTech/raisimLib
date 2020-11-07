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
  auto ground = world.addGround();
  auto hummingbird = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\hummingbird\\hummingbird.urdf");
  hummingbird->setGeneralizedCoordinate({0,0,10});
  hummingbird->setName("hummingbird");

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    std::this_thread::sleep_for(std::chrono::microseconds(500000000));
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
