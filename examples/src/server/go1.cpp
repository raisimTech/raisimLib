// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#define PI 3.1415926
#include <unistd.h>
auto ToEulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x ()* q.x() + q.y() * q.y());
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w ()* q.y ()- q.x() * q.z()));
    double cosp = std::sqrt(1 - 2 * (q.w ()* q.y() - q.x() * q.z()));
    angles[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles[2] = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void swap(Eigen::VectorXd &ob, int a, int b)
{
    double tmp = ob[a];
    ob[a] = ob[b];
    ob[b] = ob[a];
}
//void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
//{
// // todo ang2, rate, ang
////    std::cout<< "size is "<<  angle_list.size() << std::endl;
//    double base1= 0.523, base3=0.0;
//    double base2 = -2 * base1;
//    double ang = abs(sin( float(idx) / T  * PI)) * rate;
//    double ang2 = -0.15 * ang * 0;
//    double ang3 = ang2 * -2;
//    int idx_base = 0;
////    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
////    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    if ( (int(idx/T) % 2 ) == 1)
//    {
//
//        angle_list[idx_base+1] = ang + base1;
//        angle_list[idx_base+2] =  base2;
//
//        angle_list[idx_base+4] = base1 + ang2;
//        angle_list[idx_base+5] = base2 + ang3;
//
//        angle_list[idx_base+7] = base1 + ang2;
//        angle_list[idx_base+8] = base2 + ang3;
//        angle_list[idx_base+10] = ang + base1;
//        angle_list[idx_base+11] =  base2;
//    }
//    else
//    {
//
//        angle_list[idx_base+1] = base1 + ang2;
//        angle_list[idx_base+2] = base2 + ang3;
//
//        angle_list[idx_base+10] = base1 + ang2;
//        angle_list[idx_base+11] = base2 + ang3;
//        angle_list[idx_base+4] = ang + base1;
//        angle_list[idx_base+5] =  base2;
//
//        angle_list[idx_base+7] = ang + base1;
//        angle_list[idx_base+8] =  base2;
//    }
//    angle_list[idx_base+0] = base3;
//    angle_list[idx_base+3] = base3;
//    angle_list[idx_base+6] = base3;
//    angle_list[idx_base+9] = base3;
//
//}

void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
{
//    std::cout<< "size is "<<  angle_list.size() << std::endl;
    double base1= 0.523, base3=0.0;
    double base2 = -2 * base1;
    double ang = abs(sin( float(idx) / T  * PI)) * rate;

    int idx_base = 0;
//    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
//    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    if ( (int(idx/T) % 2 ) == 1)
    {

        angle_list[idx_base+1] = ang + base1;
        angle_list[idx_base+2] = -2 * ang + base2;

        angle_list[idx_base+4] = base1;
        angle_list[idx_base+5] = base2;

        angle_list[idx_base+7] = base1;
        angle_list[idx_base+8] = base2;
        angle_list[idx_base+10] = ang + base1;
        angle_list[idx_base+11] = - 2 * ang + base2;
    }
    else
    {

        angle_list[idx_base+1] = base1;
        angle_list[idx_base+2] = base2;

        angle_list[idx_base+10] = base1;
        angle_list[idx_base+11] = base2;
        angle_list[idx_base+4] = ang + base1;
        angle_list[idx_base+5] = -2 * ang + base2;

        angle_list[idx_base+7] = ang + base1;
        angle_list[idx_base+8] = -2 * ang + base2;
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

    jointNominalConfig << 0, 0, 0.33, 1.0, 0.0, 0, 0, 0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976 ; // todo implement in python using list
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
    Eigen::VectorXd gc, gv;
    Eigen::VectorXd position, velo;
    position.setZero(12), velo.setZero(12);
    gc.setZero(19), gv.setZero(18);
    size_t dof = go1->getDOF();

    raisim::Vec<3> vel;
    vel.setZero();
    raisim::Vec<3> acc;
    Eigen::Vector3d ma;
    ma.setZero();
  for (int i=0; i<2000000; i++) {
      go1->getState(gc,gv);
      Eigen::VectorXd tmp1(12), tmp2(12);
      tmp1 = gc.tail(12);
      tmp2 = gv.tail(12);
      for(auto i=0; i<12; i++){
          ob_[2*i] = tmp1[i];
          ob_[2 * i+1] = tmp2[i];
      }
      for(auto i=0;i <12; i++)
      {
          swap(ob_, i, 12 +i);
      }

      raisim::Vec<3> vel_temp;
      go1->getFrameVelocity("ROOT", vel_temp);
//      for(auto i =0;i <2; i++)
//      {
      acc = (vel_temp - vel) / 0.01;
      vel = vel_temp;
//          acc[i] = vel_temp
//      }
//      std::cout << "vel \n" << vel  << std::endl << "acc\n " << acc << std::endl;
      usleep(10000);
      angle_generator(position, i, 40, 0.15);
      jointNominalConfig.tail(12) = position;

      Eigen::Quaterniond quat(gc[3], gc[4], gc[5], gc[6]);
      Eigen::Vector3d angle = ToEulerAngles(quat);
      if (abs(angle[0]) >abs(ma[0]) ) ma[0] = abs(angle[0]);
      if (abs(angle[1]) >abs(ma[1]) ) ma[1] = abs(angle[1]);
      if (abs(angle[2]) >abs(ma[2]) ) ma[2] = abs(angle[2]);

//      std::cout<<quat << std::endl;
      std::cout<< "max" << ma << std::endl;

      RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
//      std::cout<< ob_ << std::endl<<std::endl;
 // todo acc is not easy to figure out
//a    s0.1e

//      go1->setBasePos(pos);
//      angle_generator(tmp, i, 50.f, 0.3);
//      double rrr = 0;
//      for(int i=4; i<=6 ; i++)
//      {
//          rrr += abs(jointNominalConfig[i] - ) * stable_reward_rate ;
//      }
//    jointNominalConfig.tail(12) = tmp;
//    std::cout<< tmp << std::endl;

go1->setPdTarget(jointNominalConfig, jointVelocityTarget);


    server.integrateWorldThreadSafe();

  }

  server.killServer();
}
