// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

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
  world.setTimeStep(0.001);

  /// create objects
  auto movingGround = world.addBox(10, 10, 1, 10, "gnd");
  movingGround->setBodyType(raisim::BodyType::KINEMATIC); // kinematic objects have infinite mass
  movingGround->setPosition(0,0,-0.5);
  auto anymalB = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");

  /// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(anymalB->getGeneralizedCoordinateDim()), jointVelocityTarget(anymalB->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(anymalB->getDOF()), jointDgain(anymalB->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  jointNominalConfig[1] = 1.0;
  anymalB->setGeneralizedCoordinate(jointNominalConfig);
  anymalB->setGeneralizedForce(Eigen::VectorXd::Zero(anymalB->getDOF()));
  anymalB->setPdGains(jointPgain, jointDgain);
  anymalB->setPdTarget(jointNominalConfig, jointVelocityTarget);
  anymalB->setName("anymalB");

  /// friction example. uncomment it to see the effect
  //  anymalB->getCollisionBody("LF_FOOT/0").setMaterial("LF_FOOT");
  //  world.setMaterialPairProp("gnd", "LF_FOOT", 0.01, 0, 0);

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i=0; i<20000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    movingGround->setLinearVelocity({0, 0, 3. * sin(double(i)/3000. * M_PI)});
    server.integrateWorldThreadSafe();
//    world.integrate();
  }

  server.killServer();
}
