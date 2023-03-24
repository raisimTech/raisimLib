// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.005);

  while (1) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))

    /// create objects
    auto ground = world.addGround();
    ground->setName("ground");
    ground->setAppearance("grid");
    std::vector<raisim::Box*> cubes;
    std::vector<raisim::Sphere*> spheres;
    std::vector<raisim::Capsule*> capsules;
    std::vector<raisim::Cylinder*> cylinders;

    static const int N = 6;

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
              ob->setAppearance("blue");
              break;
            case 1:
              spheres.push_back(world.addSphere(0.5, 1));
              ob = spheres.back();
              ob->setAppearance("red");
              break;
            case 2:
              capsules.push_back(world.addCapsule(0.5, 0.5, 1));
              ob = capsules.back();
              ob->setAppearance("green");
              break;
            case 3:
              cylinders.push_back(world.addCylinder(0.5, 0.5, 1));
              ob = cylinders.back();
              ob->setAppearance("0.5, 0.5, 0.8, 1.0");
              break;
          }
          ob->setPosition(-N + 2. * i, -N + 2. * j, 1. + 1.5 * k);
        }
      }
    }

    for (int i=0; i<5; i++)
      world.integrate();

    for (auto c : cubes)
      world.removeObject(c);

    for (auto c : spheres)
      world.removeObject(c);

    for (auto c : capsules)
      world.removeObject(c);

    for (auto c : cylinders)
      world.removeObject(c);
  }
}
