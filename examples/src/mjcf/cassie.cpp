// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
  timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif
  raisim::World world(binaryPath.getDirectory()+"\\rsc\\cassie\\cassie.xml");
  raisim::RaisimServer server(&world);
  auto torso = static_cast<raisim::ArticulatedSystem*>(world.getObject("cassie-pelvis"));
  torso->setBasePos({0,0,1.5});

  server.launchServer();
  for (int i=0; i<10000000; i++) {
    world.integrate();
    std::this_thread::sleep_for(std::chrono::milliseconds(size_t(10000 * world.getTimeStep())));
  }
  server.killServer();
}