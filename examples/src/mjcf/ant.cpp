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
  RSWARN("Please copy-paste files in \"https://github.com/openai/gym/tree/master/gym/envs/mujoco/assets\" to the resource directory and run cmake again (which will copy the asset folder to the build folder)")
  raisim::World world(binaryPath.getDirectory()+"\\rsc\\mjcf\\ant.xml");
  raisim::RaisimServer server(&world);
  auto torso = static_cast<raisim::ArticulatedSystem*>(world.getObject("torso"));
  torso->setBasePos({0,0,1.5});

  server.launchServer();
  for (int i=0; i<10000000; i++) {
    world.integrate();
    raisim::MSLEEP(1);
  }

  server.stopRecordingVideo();
  server.killServer();
}