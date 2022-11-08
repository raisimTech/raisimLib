// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "/rsc/activation.raisim");
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  std::unordered_map<std::string, std::string> params;
  params["spawn_sphere"] = "true";
  params["spawn_box"] = "false";
  params["sphere_count"] = "30";
  params["sphere_height_offset"] = "3";

  params["laikago_start_x"] = "-2";
  params["laikago_start_y"] = "0";
  params["floor_height"] = "-1";

  raisim::World world(binaryPath.getDirectory() + "/rsc/xmlScripts/templatedWorld/templatedWorld.xml", params);
  raisim::RaisimServer server(&world);
  server.launchServer();
  for (int i=0; i<10000000; i++) {
    server.integrateWorldThreadSafe();
    std::this_thread::sleep_for(std::chrono::milliseconds(size_t(1000 * world.getTimeStep())));
  }

  server.killServer();
}