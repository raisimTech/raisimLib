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
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.setCameraPositionAndLookAt({5,0,2}, {0,0,2});

  while (1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
