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

  std::vector<raisim::ArticulatedSystem*> atlas;

  const size_t N = 1;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      atlas.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\atlas\\robot.urdf"));
      atlas.back()->setGeneralizedCoordinate(
          {double(2 * i), double(j), 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0,           0.0,       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
      atlas.back()->setGeneralizedForce(Eigen::VectorXd::Zero(atlas.back()->getDOF()));
      atlas.back()->setName("atlas" + std::to_string(j + i * N));
      atlas.back()->setIntegrationScheme(raisim::ArticulatedSystem::IntegrationScheme::RUNGE_KUTTA_4);
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    std::this_thread::sleep_for(std::chrono::microseconds(500));
    atlas[0]->setExternalForce(0, {300,-300,30});
    atlas[0]->setExternalTorque(0, {0,40,0});
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
