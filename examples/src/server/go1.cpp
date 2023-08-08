// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#define PI 3.1415926


void swap(Eigen::VectorXd &ob, int a, int b)
{
    double tmp = ob[a];
    ob[a] = ob[b];
    ob[b] = ob[a];
}
void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
{
 // todo ang2, rate, ang
//    std::cout<< "size is "<<  angle_list.size() << std::endl;
    double base1= 0, base3=0.0;
    double base2 = -2 * base1;
    double ang = abs(sin( float(idx) / T  * PI)) * rate;
    double ang2 = -0.15 * ang * 0;
    double ang3 = ang2 * 1.2;
    int idx_base = 0;
//    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
//    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    if ( (int(idx/T) % 2 ) == 1)
    {

        angle_list[idx_base+1] = ang + base1;
        angle_list[idx_base+2] =  base2;

        angle_list[idx_base+4] = base1 + ang2;
        angle_list[idx_base+5] = base2 + ang3;

        angle_list[idx_base+7] = base1 + ang2;
        angle_list[idx_base+8] = base2 + ang3;
        angle_list[idx_base+10] = ang + base1;
        angle_list[idx_base+11] =  base2;
    }
    else
    {

        angle_list[idx_base+1] = base1 + ang2;
        angle_list[idx_base+2] = base2 + ang3;

        angle_list[idx_base+10] = base1 + ang2;
        angle_list[idx_base+11] = base2 + ang3;
        angle_list[idx_base+4] = ang + base1;
        angle_list[idx_base+5] =  base2;

        angle_list[idx_base+7] = ang + base1;
        angle_list[idx_base+8] =  base2;
    }
    angle_list[idx_base+0] = base3;
    angle_list[idx_base+3] = base3;
    angle_list[idx_base+6] = base3;
    angle_list[idx_base+9] = base3;

}


int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.01);

  /// create objects
  auto ground = world.addGround();
  auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\a1_description\\urdf\\a1.urdf");

  /// go1 joint PD controller
  Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim()), jointVelocityTarget(go1->getDOF());
//  jointNominalConfig << 0, 0, 0.41, 1.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    jointNominalConfig << 0, 0, 0.36, 1.0, 0.0, 0, 0, 0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976 ; // todo implement in python using list
//  jointNominalConfig.tail(12).setZero();
//  Eigen::VectorXd tmp(12);
//  tmp.setZero();
//    angle_generator(tmp, 0, 80.f);
//    jointNominalConfig.tail(12) = tmp;
  jointVelocityTarget.setZero();

  Eigen::VectorXd jointPgain(go1->getDOF()), jointDgain(go1->getDOF());
  jointPgain.tail(12).setConstant(2000.0);
  jointDgain.tail(12).setConstant(100);

  go1->setGeneralizedCoordinate(jointNominalConfig);
  go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF()));
  go1->setPdGains(jointPgain, jointDgain);
  go1->setPdTarget(jointNominalConfig, jointVelocityTarget);
  go1->setName("go1");
  std::cout<< go1->getGeneralizedCoordinate();
  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.focusOn(go1);
  server.launchServer();
    Eigen::Vector3d pos(0,0, 3);
    Eigen::VectorXd ob_(24);
    ob_.setZero();
  for (int i=0; i<2000000; i++) {
      go1->getState(jointNominalConfig,jointVelocityTarget);
      Eigen::VectorXd tmp1(12), tmp2(12);
      tmp1 = jointNominalConfig.tail(12);
      tmp2 = jointVelocityTarget.tail(12);
      for(auto i=0; i<12; i++){
          ob_[2*i] = tmp1[i];
          ob_[2 * i+1] = tmp2[i];
      }
      for(auto i=0;i <12; i++)
      {
          swap(ob_, i, 12 +i);
      }


      RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
      std::cout<< ob_ << std::endl<<std::endl;
      sleep(1);
//      go1->setBasePos(pos);
//      angle_generator(tmp, i, 50.f, 0.3);
//      double rrr = 0;
//      for(int i=4; i<=6 ; i++)
//      {
//          rrr += abs(jointNominalConfig[i] - ) * stable_reward_rate ;
//      }
//    jointNominalConfig.tail(12) = tmp;
//    std::cout<< tmp << std::endl;
//    go1->setPdTarget(jointNominalConfig, jointVelocityTarget);


    server.integrateWorldThreadSafe();

  }

  server.killServer();
}
