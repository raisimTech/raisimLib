// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");
  raisim::RaiSimMsg::setFatalCallback([](){throw;});

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  auto ground = world.addGround(0, "gnd");
  ground->setAppearance("hidden");
  auto anymalB = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");
  auto anymalC = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\anymal_c\\urdf\\anymal.urdf");

  /// anymalC joint PD controller
  Eigen::VectorXd jointNominalConfig(anymalC->getGeneralizedCoordinateDim()), jointVelocityTarget(anymalC->getDOF());
  jointNominalConfig << 0, 0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(anymalC->getDOF()), jointDgain(anymalC->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(1.0);

  anymalC->setGeneralizedCoordinate(jointNominalConfig);
  anymalC->setGeneralizedForce(Eigen::VectorXd::Zero(anymalC->getDOF()));
  anymalC->setPdGains(jointPgain, jointDgain);
  anymalC->setPdTarget(jointNominalConfig, jointVelocityTarget);
  anymalC->setName("anymalC");

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
  server.setMap("wheat");
  server.launchServer();
  server.focusOn(anymalC);

  /// graphs
  std::vector<std::string> jointNames = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                         "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  auto jcGraph = server.addTimeSeriesGraph("joint position", jointNames, "time", "position");
  auto jvGraph = server.addTimeSeriesGraph("joint velocity", jointNames, "time", "velocity");
  auto jfGraph = server.addTimeSeriesGraph("joint torque", jointNames, "time", "torque");

  raisim::VecDyn jc(12), jv(12), jf(12);

  for (int i=0; i<200000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
    if (i % 10 == 0) {
      jc = anymalC->getGeneralizedCoordinate().e().tail(12);
      jv = anymalC->getGeneralizedVelocity().e().tail(12);
      jf = anymalC->getGeneralizedForce().e().tail(12);
      jcGraph->addDataPoints(world.getWorldTime(), jc);
      jvGraph->addDataPoints(world.getWorldTime(), jv);
      jfGraph->addDataPoints(world.getWorldTime(), jf);
    }
  }

  server.killServer();
}
