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
  world.addGround(0, "steel");
  auto sphere1 = world.addSphere(0.5, 1.0, "steel");
  auto sphere2 = world.addSphere(0.5, 1.0, "rubber");
  auto sphere3 = world.addSphere(0.5, 1.0, "copper");

  sphere1->setPosition(-2,0,5);
  sphere2->setPosition(0,0,5);
  sphere3->setPosition(2,0,5);

  world.setMaterialPairProp("steel", "steel", 0.8, 0.95, 0.001);
  world.setMaterialPairProp("steel", "rubber", 0.8, 0.15, 0.001);
  world.setMaterialPairProp("steel", "copper", 0.8, 0.65, 0.001);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i = 0; i < 500; i++) {
    raisim::MSLEEP(1);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
