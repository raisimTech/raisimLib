// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world(binaryPath.getDirectory() + "\\rsc\\raisimUnrealMaps\\office1.xml");
  world.setTimeStep(0.001);


  /// create objects
  auto ground = world.addGround(0.02);
  ground->setAppearance("hidden");
  auto ball = world.addSphere(0.3, 2.0);
  ball->setPosition(raisim::Vec<3>{0,0,2});
  ball->setLinearVelocity(raisim::Vec<3>{1,1,0});

  auto aliengo = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\aliengo\\aliengo.urdf");

  /// aliengo joint PD controller
  Eigen::VectorXd jointNominalConfig(aliengo->getGeneralizedCoordinateDim()), jointVelocityTarget(aliengo->getDOF());
  jointNominalConfig << 0, 0, 1.24, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(aliengo->getDOF()), jointDgain(aliengo->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  aliengo->setGeneralizedCoordinate(jointNominalConfig);
  aliengo->setGeneralizedForce(Eigen::VectorXd::Zero(aliengo->getDOF()));
  aliengo->setPdGains(jointPgain, jointDgain);
  aliengo->setPdTarget(jointNominalConfig, jointVelocityTarget);
  aliengo->setName("aliengo");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.setMap("office1");
  server.focusOn(aliengo);
  server.launchServer();

  for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
