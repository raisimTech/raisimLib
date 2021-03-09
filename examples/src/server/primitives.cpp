// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  /// create raisim world
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
#if WIN32
    timeBeginPeriod(1); // for sleep_for function. windows default clock speed is 1/64 second. This sets it to 1ms.
#endif

  raisim::World world;
  world.setTimeStep(0.002);

  /// create objects
  auto ground = world.addGround();

  std::vector<raisim::Box*> cubes;
  std::vector<raisim::Sphere*> spheres;
  std::vector<raisim::Capsule*> capsules;
  std::vector<raisim::Cylinder*> cylinders;

  static const int N = 3;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      for (size_t k = 0; k < N; k++) {
        std::string number =
            std::to_string(i) + std::to_string(j) + std::to_string(k);
        raisim::SingleBodyObject* ob = nullptr;
        switch ((i + j + k) % 4) {
          case 0:
            cubes.push_back(world.addBox(1, 1, 1, 1));
            ob = cubes.back();
            break;
          case 1:
            spheres.push_back(world.addSphere(0.5, 1));
            ob = spheres.back();
            break;
          case 2:
            capsules.push_back(world.addCapsule(0.5, 1., 1));
            ob = capsules.back();
            break;
          case 3:
            cylinders.push_back(world.addCylinder(0.5, 0.5, 1));
            ob = cylinders.back();
            break;
        }
        ob->setPosition(-N + 2. * i, -N + 2. * j, N * 2. + 2. * k);
      }
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
