// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create raisim objects
  auto ground = world.addGround();

  // simulator
  world.setMaterialPairProp("steel", "steel", 0.1, 1.0, 0.0);

  auto pin1 = world.addSphere(0.1, 0.8);
  pin1->setPosition(0.0, 0.0, 3.0);
  pin1->setBodyType(raisim::BodyType::STATIC);

  auto pin2 = world.addSphere(0.1, 0.8);
  pin2->setPosition(0.3, 0.0, 3.0);
  pin2->setBodyType(raisim::BodyType::STATIC);

  auto pin3 = world.addSphere(0.1, 0.8);
  pin3->setPosition(0.6, 0.0, 3.0);
  pin3->setBodyType(raisim::BodyType::STATIC);

  auto pin4 = world.addSphere(0.1, 0.8);
  pin4->setPosition(0.9, 0.0, 3.0);
  pin4->setBodyType(raisim::BodyType::STATIC);

  auto pin5 = world.addSphere(0.1, 0.8);
  pin5->setPosition(0.9, 0.0, 6.0);
  pin5->setBodyType(raisim::BodyType::STATIC);

  auto ball1 = world.addSphere(0.1498, 0.8, "steel");
  ball1->setPosition(0, 0.0, 1.0);

  auto ball2 = world.addSphere(0.1499, 0.8, "steel");
  ball2->setPosition(0.3, 0.0, 1.0);

  auto ball3 = world.addSphere(0.1499, 0.8, "steel");
  ball3->setPosition(0.6, 0.0, 1.0);

  auto ball4 = world.addSphere(0.1499, 0.8, "steel");
  ball4->setPosition(2.9, 0.0, 3.0);

  auto box = world.addBox(.1, .1, .1, 1);
  box->setPosition(0.9, 0.0, 4.0);

  auto wire1 = world.addStiffWire(pin1, 0, {0,0,0}, ball1, 0, {0,0,0}, 2.0);
  world.addStiffWire(pin2, 0, {0,0,0}, ball2, 0, {0,0,0}, 2.0);
  world.addStiffWire(pin3, 0, {0,0,0}, ball3, 0, {0,0,0}, 2.0);
  world.addStiffWire(pin4, 0, {0,0,0}, ball4, 0, {0,0,0}, 2.0);

  auto wire5 = world.addCompliantWire(pin5, 0, {0,0,0}, box, 0, {0., 0, 0}, 2.0, 200);

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim::MSLEEP(1000);

  while (1) {
    raisim::MSLEEP(1);
    server.integrateWorldThreadSafe();
  }
}