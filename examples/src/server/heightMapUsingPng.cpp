// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>
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
  auto heightMap = world.addHeightMap(binaryPath.getDirectory() + "\\rsc\\xmlScripts\\heightMaps\\zurichHeightMap.png", 0, 0, 500, 500, 0.005, -10);
  auto anymal = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");
  anymal->setGeneralizedCoordinate({0, 0, 10.8, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
  anymal->setName("anymal");
  heightMap->setAppearance("soil1");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i = 0; i < 50000; i++) {
    raisim::MSLEEP(1);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}