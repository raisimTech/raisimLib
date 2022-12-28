// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.002);

  /// create objects
  auto ground = world.addGround();
  std::vector<raisim::Compound::CompoundObjectChild> children;

  /// just to get random motions of anymal
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 0.6);
  std::srand(std::time(nullptr));

  for(int i=0; i<20; i++) {
    raisim::Compound::CompoundObjectChild child;
    child.objectType = raisim::ObjectType::CAPSULE;
    child.objectParam[0] = 0.1; // radius
    child.objectParam[1] = 0.1; // height (center-to-center distance)
    child.trans.pos[0] = distribution(generator);
    child.trans.pos[1] = distribution(generator);
    child.trans.pos[2] = distribution(generator);
    raisim::Vec<4> quat;
    quat[0] = distribution(generator);
    quat[1] = distribution(generator);
    quat[2] = distribution(generator);
    quat[3] = distribution(generator);
    quat /= quat.norm();
    raisim::quatToRotMat(quat, child.trans.rot);
    children.push_back(child);
  }

  raisim::Mat<3,3> inertia; inertia.setIdentity();
  auto compound = world.addCompound(children, 5, {0,0,0}, inertia);
  compound->setPosition(0, 0, 3);
  compound->setAppearance("1,0,0,0.3");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
