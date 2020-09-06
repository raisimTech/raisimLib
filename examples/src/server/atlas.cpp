// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.002);

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
      atlas.back()->setGeneralizedForce(
          Eigen::VectorXd::Zero(atlas.back()->getDOF()));
      atlas.back()->setName("atlas" + std::to_string(j + i * N));
    }
  }

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
