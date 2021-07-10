// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
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
  world.setTimeStep(0.002);

  /// create objects
  raisim::TerrainProperties terrainProperties;
  terrainProperties.frequency = 0.2;
  terrainProperties.zScale = 3.0;
  terrainProperties.xSize = 20.0;
  terrainProperties.ySize = 20.0;
  terrainProperties.xSamples = 50;
  terrainProperties.ySamples = 50;
  terrainProperties.fractalOctaves = 3;
  terrainProperties.fractalLacunarity = 2.0;
  terrainProperties.fractalGain = 0.25;
  raisim::HeightMap* hm = world.addHeightMap(0.0, 0.0, terrainProperties);
  hm->setAppearance("blue");

  std::vector<raisim::ArticulatedSystem*> anymals;

  /// ANYmal joint PD controller
  Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
  Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18),
      jointDgain(18);
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  jointPgain.tail(12).setConstant(200.0);
  jointDgain.tail(12).setConstant(10.0);

  const size_t N = 4;

  for (size_t i = 0; i < N; i++) {
    for (size_t j = 0; j < N; j++) {
      anymals.push_back(world.addArticulatedSystem(
          binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf"));
      anymals.back()->setGeneralizedCoordinate(
          {double(2 * i)-1, double(j), 2.5, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
           -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8});
      anymals.back()->setGeneralizedForce(
          Eigen::VectorXd::Zero(anymals.back()->getDOF()));
      anymals.back()->setControlMode(
          raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
      anymals.back()->setPdGains(jointPgain, jointDgain);
      anymals.back()->setName("anymal" + std::to_string(j + i * N));
    }
  }

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i=0; i<1000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
