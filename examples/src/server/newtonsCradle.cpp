// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main(int argc, char **argv) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create raisim objects
  auto ground = world.addGround();

  // simulator
  world.setMaterialPairProp("steel", "steel", 0.1, 1.0, 0.0);

  auto pin1 = world.addSphere(0.1, 0.8);
  pin1->setAppearance("1,0,0,0.3");
  pin1->setPosition(0.0, 0.0, 3.0);
  pin1->setBodyType(raisim::BodyType::STATIC);

  auto pin2 = world.addSphere(0.1, 0.8);
  pin2->setAppearance("0,1,0,0.3");
  pin2->setPosition(0.3, 0.0, 3.0);
  pin2->setBodyType(raisim::BodyType::STATIC);

  auto pin3 = world.addSphere(0.1, 0.8);
  pin3->setAppearance("0,0,1,0.3");
  pin3->setPosition(0.6, 0.0, 3.0);
  pin3->setBodyType(raisim::BodyType::STATIC);

  auto pin4 = world.addSphere(0.1, 0.8);
  pin4->setAppearance("1,0,0,0.3");
  pin4->setPosition(0.9, 0.0, 3.0);
  pin4->setBodyType(raisim::BodyType::STATIC);

  auto pin5 = world.addSphere(0.1, 0.8);
  pin5->setPosition(0.9, 0.0, 6.0);
  pin5->setBodyType(raisim::BodyType::STATIC);

  auto pin6 = world.addSphere(0.1, 0.8);
  pin6->setPosition(-3., 0.0, 7.0);
  pin6->setBodyType(raisim::BodyType::STATIC);

  auto pin7 = world.addSphere(0.1, 0.8);
  pin7->setPosition(-4., 0.0, 7.0);
  pin7->setBodyType(raisim::BodyType::STATIC);

  auto anymalC = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal.urdf");
  auto anymalB = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");

  /// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(anymalC->getGeneralizedCoordinateDim()), jointVelocityTarget(anymalC->getDOF());
  jointNominalConfig << -3, 0, 4.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(anymalC->getDOF()), jointDgain(anymalC->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  anymalC->setGeneralizedCoordinate(jointNominalConfig);
  anymalC->setGeneralizedForce(Eigen::VectorXd::Zero(anymalC->getDOF()));
  anymalC->setPdGains(jointPgain, jointDgain);
  anymalC->setPdTarget(jointNominalConfig, jointVelocityTarget);
  anymalC->setName("anymalC");

  jointNominalConfig[0] = -4;
  anymalB->setGeneralizedCoordinate(jointNominalConfig);
  anymalB->setGeneralizedForce(Eigen::VectorXd::Zero(anymalB->getDOF()));
  anymalB->setPdGains(jointPgain, jointDgain);
  anymalB->setPdTarget(jointNominalConfig, jointVelocityTarget);
  anymalB->setName("anymalB");

  auto ball1 = world.addSphere(0.1498, 0.8, "steel");
  ball1->setPosition(0, 0.0, 1.0);

  auto ball2 = world.addSphere(0.1499, 0.8, "steel");
  ball2->setPosition(0.3, 0.0, 1.0);

  auto ball3 = world.addSphere(0.1499, 0.8, "steel");
  ball3->setPosition(0.6, 0.0, 1.0);

  auto ball4 = world.addSphere(0.1499, 0.8, "steel");
  ball4->setPosition(2.9, 0.0, 3.0);

  auto box = world.addBox(.1, .1, .1, 1);
  box->setPosition(0.9, 0.0, 4.2);

  auto wire1 = world.addStiffWire(pin1, 0, {0,0,0}, ball1, 0, {0,0,0}, 2.0);
  wire1->setVisualizationWidth(0.05);
  wire1->setColor({1, 0, 0, 1});
  world.addStiffWire(pin2, 0, {0,0,0}, ball2, 0, {0,0,0}, 2.0);
  world.addStiffWire(pin3, 0, {0,0,0}, ball3, 0, {0,0,0}, 2.0);
  world.addStiffWire(pin4, 0, {0,0,0}, ball4, 0, {0,0,0}, 2.0);

  auto wire5 = world.addCompliantWire(pin5, 0, {0,0,0}, box, 0, {0., 0, 0}, 2.0, 200);
  wire5->setStretchType(raisim::LengthConstraint::StretchType::BOTH);

  auto wire6 = world.addCompliantWire(pin6, 0, {0,0,0}, anymalC, 0, {0., 0, 0}, 2.0, 1000);
  wire6->setStretchType(raisim::LengthConstraint::StretchType::BOTH);

  auto wire7 = world.addCustomWire(pin7, 0, {0,0,0}, anymalB, 0, {0., 0, 0}, 2.0);
  wire7->setTension(310);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer(8080);

  world.exportToXml(binaryPath.getDirectory(), "exportedWorld.xml");

  for (int i=0; i< 5000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    if (server.isConnected()) {
      server.integrateWorldThreadSafe();
    }

    if (i == 5000)
      world.removeObject(wire7);
  }

  server.killServer();
}