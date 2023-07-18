// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  auto ground = world.addGround();
  auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\go1\\go1.urdf");

  /// go1 joint PD controller
  Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim()), jointVelocityTarget(go1->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(go1->getDOF()), jointDgain(go1->getDOF());
  jointPgain.tail(12).setConstant(10.0);
  jointDgain.tail(12).setConstant(0.1);

  go1->setGeneralizedCoordinate(jointNominalConfig);
  go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF()));
  go1->setPdGains(jointPgain, jointDgain);
  go1->setPdTarget(jointNominalConfig, jointVelocityTarget);
  go1->setName("go1");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.focusOn(go1);
  server.launchServer();

  for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
