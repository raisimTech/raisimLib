// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.003);
  world.setERP(world.getTimeStep(), world.getTimeStep());

  /// create objects
  auto ground = world.addGround();

  std::string monkeyFile =
      binaryPath.getDirectory() + "\\rsc\\monkey\\monkey.obj";

  raisim::Mat<3, 3> inertia;
  inertia.setIdentity();
  const raisim::Vec<3> com = {0, 0, 0};

  int N = 3;
  double gap = 1;

  for (int row = 0; row < N; row++) {
    for (int col = 0; col < N; col++) {
      auto monkey = world.addMesh(monkeyFile, 1.0, inertia, com);
      monkey->setPosition(-gap * (N / 2) + gap * row,
                          -gap * (N / 2) + gap * col,
                          2.0 + gap * (row * N + col));
      monkey->setAppearance("blue");
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.setCameraPositionAndLookAt({5,0,2}, {0,0,2});

  while (1) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
