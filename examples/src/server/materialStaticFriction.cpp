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
  auto box = world.addBox(1,1,1,1,"steel");
  auto ground = world.addGround(0, "brass");
  box->setPosition(0,0,0.5);

  /// option 1: use cpp api to define the materials
//  world.setMaterialPairProp("steel", "brass", 0.2, 0., 0., 0.95, 0.01);
  /// option 2: use the material xml file
  world.updateMaterialProp(raisim::MaterialManager(binaryPath.getDirectory() + "\\rsc\\testMaterials.xml"));

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(box);

  std::cout<<"static friction is active because the initial velocity is too low."<<std::endl;
  for (int i = 0; i < 5000; i++) {
    raisim::MSLEEP(1);
    box->setExternalForce(0, {5, 0, 0});
    server.integrateWorldThreadSafe();
  }

  box->setLinearVelocity({0.01, 0, 0});
  std::cout<<"now dynamic friction is active"<<std::endl;

  for (int i = 0; i < 5000; i++) {
    raisim::MSLEEP(1);
    box->setExternalForce(0, {5, 0, 0});
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
