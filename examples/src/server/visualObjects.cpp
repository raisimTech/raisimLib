// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.003);

  /// create and launch raisim server
  raisim::RaisimServer server(&world);

  /// ground
  auto ground = world.addGround();

  /// primitives
  auto visSphere = server.addVisualSphere("v_sphere", 1.0, 1, 1, 1, 0.5);
  auto visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1);
  auto visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1);
  auto visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1);
  raisim::Mat<3, 3> inertia;
  inertia.setIdentity();
  const raisim::Vec<3> com = {0, 0, 0};
  auto mesh = world.addMesh(binaryPath.getDirectory() + "/rsc/monkey/monkey.obj", 1, inertia, com);
  auto visMesh = server.addVisualMesh("v_mesh", binaryPath.getDirectory() + "/rsc/monkey/monkey.obj");
  auto varrow_x = server.addVisualArrow("v_arrow_x", 1, 2, 1, 0, 0, 1);
  auto varrow_y = server.addVisualArrow("v_arrow_y", 1, 2, 0, 1, 0, 1);
  auto varrow_z = server.addVisualArrow("v_arrow_z", 1, 2, 0, 0, 1, 1);

  /// positions and orientations
  raisim::Vec<4> quat = {0.6, 0.2, -0.6, 0.1};
  quat = quat / quat.norm();
  mesh->setOrientation(quat);
  visMesh->setOrientation(quat.e());
  mesh->setPosition(2,-2,1.1);
  visSphere->setPosition(2,0,0);
  visCylinder->setPosition(0,2,0);
  visCapsule->setPosition(2,2,0);
  visMesh->setPosition(2,-2,1);
  varrow_x->setPosition(0,0,4);
  varrow_y->setPosition(0,0,4);
  varrow_z->setPosition(0,0,4);
  Eigen::Vector4d xDir = {0.70710678118, 0, 0.70710678118, 0};
  Eigen::Vector4d yDir = {0.70710678118, -0.70710678118, 0, 0};
  varrow_x->setOrientation(xDir);
  varrow_y->setOrientation(yDir);

  /// articulated systems
  auto anymalB = server.addVisualArticulatedSystem("v_anymal", binaryPath.getDirectory() + "/rsc/anymal/urdf/anymal.urdf");
  anymalB->color = {0.5, 0.0, 0.0, 0.5};
  Eigen::VectorXd gc(19);
  gc << 0, 0, 5.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
  anymalB->setGeneralizedCoordinate(gc);

  /// poly lines
  auto lines = server.addVisualPolyLine("lines");
  lines->color = {0,0,1,1};
  auto lines2 = server.addVisualPolyLine("lines2");
  lines2->color = {0,0,1,1};

  for( int i = 0; i < 100; i++)
    lines->points.push_back({sin(i*0.1), cos(i*0.1), i*0.01});

  for( int i = 0; i < 100; i++)
    lines2->points.push_back({sin(i*0.1)+2, cos(i*0.1), i*0.01});

  /// dynamic mesh
  std::vector<float> vertices;
  std::vector<int32_t> index;
  std::vector<uint8_t> color;
  vertices = {0.5f,0.5f,0.f, 0.0f,-1.f,0.f, -0.5f,0.5f,0.f, 0.f,0.f,1.f};
  index = {0,3,1, 1,3,2, 2,3,0, 0,1,2};
  color = {255,0,0, 0,255,0, 0,0,255, 0,0,126};
  auto dynamicMesh = server.addVisualMesh("custom mesh", vertices, color, index, 1., 1., 1., 1.);
  dynamicMesh->setPosition(-2,0,0);

  server.launchServer();
  size_t counter = 0;
  for (int i=0; i<500000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    counter++;

    server.lockVisualizationServerMutex();
    visBox->color[2] = double((counter)%255+1)/256.;
    visBox->setBoxSize(1, double((counter)%255+1)/256.+0.01, 1);
    visCapsule->setCapsuleSize(double((counter)%255+1)/256.+0.01, 1);
    visCylinder->setCapsuleSize(double((counter)%255+1)/256.+0.01, 1);
    visSphere->color[1] = double((counter)%255+1)/256.;
    lines->color[2] = double((counter)%255+1)/256.;
    lines->color[0] = 1. - lines->color[2];

    vertices[0] = 0.5f * (2.+sin(counter*0.01));
    color[10] = uint8_t(counter/5%255);
    dynamicMesh->updateMesh(vertices, color);

    server.unlockVisualizationServerMutex();
//    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
