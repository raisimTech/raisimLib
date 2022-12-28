// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.002);

  /// create objects
  raisim::TerrainProperties terrainProperties;
  terrainProperties.frequency = 0.2;
  terrainProperties.zScale = 3.0;
  terrainProperties.xSize = 20.0;
  terrainProperties.ySize = 20.0;
  terrainProperties.xSamples = 50;
  terrainProperties.ySamples = 50;
  terrainProperties.fractalOctaves = 3;
  terrainProperties.fractalLacunarity = 2.0;
  terrainProperties.fractalGain = 0.25;

  auto hm = world.addHeightMap(0.0, 0.0, terrainProperties);
  hm->setAppearance("soil1");
  auto cube = world.addBox(1,1,1,1);
  cube->setPosition(3,0,3);
  cube->setAppearance("blue");
  auto cylinder = world.addCylinder(1, 1, 1);
  cylinder->setPosition(3,3,3);
  cylinder->setAppearance("yellow");
  auto capsule = world.addCapsule(1, 1, 1);
  capsule->setPosition(-3,3,3);
  auto sphere = world.addSphere(1, 1);
  sphere->setPosition(-3,0,3);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  int counter=0;
  auto line = server.addVisualPolyLine("ray");
  line->color = {1,0,0,1};
  line->points.push_back({0,0,5});
  line->points.push_back({0,0,0});
  line->width = 0.05;
  auto visSphere = server.addVisualSphere("viz_sphere", 0.3, 1,0,0,1);

  while (1) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
    double angle = counter/1000.;
    double magnitude = (counter%3000)*0.1;
    Eigen::Vector3d direction = {cos(angle)*magnitude, sin(angle)*magnitude, -100};
    direction /= direction.norm();
    auto& col = world.rayTest({0,0,5}, direction, 50., true);
    if(col.size() > 0) {
      line->points[1] = col[0].getPosition();
      visSphere->setPosition(col[0].getPosition()[0], col[0].getPosition()[1], col[0].getPosition()[2]);
    }
    counter++;
  }

  server.killServer();
}
