// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "\\rsc\\activation.raisim");

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.003);

  /// create objects
  auto ground = world.addGround();

  /// launch raisim servear
  raisim::RaisimServer server(&world);
  server.launchServer();

  auto visSphere = server.addVisualSphere("v_sphere", 1.0, 1, 1, 1, 1);
  auto visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1);
  auto visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1);
  auto visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1);
  auto anymalB = server.addVisualArticulatedSystem("v_anymal", binaryPath.getDirectory() + "\\rsc\\anymal\\urdf\\anymal.urdf");

  visSphere->setPosition(2,0,0);
  visCylinder->setPosition(0,2,0);
  visCapsule->setPosition(2,2,0);
  Eigen::VectorXd gc(19);
  gc << 0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  anymalB->setGeneralizedCoordinate(gc);
  anymalB->color = {0.5,0.0,0.0,0.5};
  auto lines = server.addVisualPolyLine("lines");
  lines->color = {0,0,1,1};

  for( int i = 0; i < 100; i++)
    lines->points.push_back({sin(i*0.1), cos(i*0.1), i*0.01});

  size_t counter = 0;
  for (int i=0; i<5000; i++) {
    counter++;
    visBox->color[2] = double((counter)%255+1)/256.;
    visBox->setBoxSize(double((counter)%255+1)/256.+0.01, 1, 1);
    visSphere->color[1] = double((counter)%255+1)/256.;
    raisim::MSLEEP(2);

    lines->color[2] = double((counter)%255+1)/256.;
    lines->color[0] = 1. - lines->color[2];
//    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
