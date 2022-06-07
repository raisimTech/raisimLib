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
  world.setERP(0,0);

  /// create objects
  auto ground = world.addGround();
  ground->setAppearance("dune"); // this works only in raisimUnreal

  std::vector<raisim::ArticulatedSystem*> atlas;

  const size_t N = 1;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      atlas.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf"));
      atlas.back()->setGeneralizedCoordinate(
          {double(2 * i), double(j), 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      atlas.back()->setGeneralizedForce(Eigen::VectorXd::Zero(atlas.back()->getDOF()));
      atlas.back()->setName("atlas" + std::to_string(j + i * N));
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);

  /// use raisimUnreal to visualize the charts
  auto timeSeries = server.addTimeSeriesGraph("body pos", {"atlas_x", "atlas_y", "atlas_z", "w", "x", "y", "z"}, "time", "pos");
  auto barChart = server.addBarChart("body pos2", {"x", "y", "z"});
  barChart->setData({0.1, 0.2, 0.3});

  server.launchServer();

  while (1) {
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    atlas[0]->setExternalForce(0, {300,-300,30});
    atlas[0]->setExternalTorque(0, {0,40,0});
    raisim::VecDyn vec(7);
    vec[0] = atlas[0]->getGeneralizedCoordinate()[0];
    vec[1] = atlas[0]->getGeneralizedCoordinate()[1];
    vec[2] = atlas[0]->getGeneralizedCoordinate()[2];
    vec[3] = atlas[0]->getGeneralizedCoordinate()[3];
    vec[4] = atlas[0]->getGeneralizedCoordinate()[4];
    vec[5] = atlas[0]->getGeneralizedCoordinate()[5];
    vec[6] = atlas[0]->getGeneralizedCoordinate()[6];

    timeSeries->addDataPoints(world.getWorldTime(), vec);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
