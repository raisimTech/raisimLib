// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char *argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World world;
  world.setTimeStep(0.01);
  std::vector<double> height(10000, 0.);
  std::vector<raisim::ColorRGB> colorMap(10000, {0, 0, 0});
  colorMap.resize(10000);
  raisim::Vec<3> color1{1,0,0}, color2{0,0,1};

  auto hm = world.addHeightMap(100, 100, 10., 10., 0., 0., height);
  hm->setName("dynamic_heightmap");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i=0; i<1000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))

    for (int k=0; k<100; k++) {
      for (int j=0; j<100; j++) {
        height[k*100+j] = std::sin(0.01f * float(i) + 0.05 * float(k+j));
        colorMap[k*100+j] = {uint8_t((std::sin(0.02f * float(i) + 0.05 * float(2*k+j)) * 0.5f + 0.5f)*255.f),
                           uint8_t((std::cos(0.02f * float(i) + 0.07 * float(2*k+j)) * 0.5f + 0.5f)*255.f),
                           uint8_t((std::sin(0.02f * float(i) + 0.09 * float(2*k+j)) * 0.5f + 0.5f)*255.f)};
      }
    }
    server.lockVisualizationServerMutex();
    hm->update(0., 0., 10., 10., height);
    hm->setColor(colorMap);

    world.integrate();
    server.unlockVisualizationServerMutex();
  }

  server.killServer();

  return 0;
}