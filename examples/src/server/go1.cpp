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

void rot2euler( const raisim::Mat<3,3>& mat, Eigen::Vector3d& euler)
{
    euler[0] = atan2(mat[2], mat[8]);
    euler[1] = asin(- mat[5]);
    euler[2] = atan2(mat[3], mat[4]);
}

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
  char c;
  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.01);

  /// create objects
  auto ground = world.addGround(0, "land");
  auto go1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\a1_description\\urdf\\a1.urdf");
  auto board = world.addArticulatedSystem(binaryPath.getDirectory() +"\\rsc\\skate\\skate.urdf");
  /// go1 joint PD controller
  Eigen::VectorXd jointNominalConfig(go1->getGeneralizedCoordinateDim()), jointVelocityTarget(go1->getDOF());
//  jointNominalConfig << 0, 0, 0.41, 1.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd board_posi(board->getGeneralizedCoordinateDim());
    board_posi << 0,0,0.11, 0.707, 0,0, 0.707,0,0;
    board->setGeneralizedCoordinate(board_posi);
    board->setName("board");
    jointNominalConfig << -0, 0, 0.49, 1.0, 0.0, 0, 0, 0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976, 0.0, 0.5235987755982988, -1.0471975511965976 ; // todo implement in python using list
//    jointNominalConfig << 0, 0, 0.33, 1.0, 0.0, 0, 0, 0, 1, -2.2, 0.0, 1, -2.2, 0.0, 1, -2.2, 0.0, 1, -2.2; // todo implement in python using list
//  jointNominalConfig.tail(12).setZero();
//  Eigen::VectorXd tmp(12);
//  tmp.setZero();
//    angle_generator(tmp, 0, 80.f);
//    jointNominalConfig.tail(12) = tmp;
  jointVelocityTarget.setZero();
    go1->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  Eigen::VectorXd jointPgain(go1->getDOF()), jointDgain(go1->getDOF());
  jointPgain.tail(12).setConstant(80);
  jointDgain.tail(12).setConstant(4);

  go1->setGeneralizedCoordinate(jointNominalConfig);
  go1->setGeneralizedForce(Eigen::VectorXd::Zero(go1->getDOF()));
  go1->setPdGains(jointPgain, jointDgain);
  go1->setPdTarget(jointNominalConfig, jointVelocityTarget);
  go1->setName("go1");
  std::cout<< go1->getGeneralizedCoordinate();

    for(auto n : go1->getCollisionBodies())
    {
        auto name = n.colObj->name;
        go1->getCollisionBody(name).setMaterial("steel");
//        go1->getCollisionBody()
    }

    for(auto n : board->getCollisionBodies())
    {
        auto name = n.colObj->name;
        std::cout << name << std::endl;
//        board->getCollisionBody(name).setMaterial("steel");
//        go1->getCollisionBody()
    }
//    wrl
    board->getCollisionBody("base/0").setMaterial("sandpaper");
    board->getCollisionBody("rotater_r/0").setMaterial("rubber");
    board->getCollisionBody("rotater_f/0").setMaterial("rubber");
    go1->getCollisionBody("FL_foot/0").setMaterial("rubber");
    go1->getCollisionBody("FR_foot/0").setMaterial("rubber");
    go1->getCollisionBody("RR_foot/0").setMaterial("rubber");
    go1->getCollisionBody("RL_foot/0").setMaterial("rubber");
    world.setMaterialPairProp("steel", "rubber", 0.8, 0.15, 0.001);
    world.setMaterialPairProp("rubber", "sandpaper", 0.99, 0.15, 0.001);
    world.setMaterialPairProp("land", "rubber", 0.8, 0.1,0.001);
    world.setMaterialPairProp("sandpaper", "land", 0.4, 0.15,0.001);
    world.setMaterialPairProp("steel","land", 0.1, 0.05,0.001);
//    world.updateMaterialProp();
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
//    go1->set


    raisim::Vec<3> vel;
    vel.setZero();
    raisim::Vec<3> acc;
    Eigen::Vector3d ma;
    ma.setZero();
    Eigen::VectorXd  bodyLinearVelocity, bodyAngularVelocity;
    bodyAngularVelocity.setZero(3);
    bodyLinearVelocity.setZero(3);
    auto roott = go1->getBodyIdx("base");
//    go1->getCollisionBodies();
//    auto imu = go1->getSensor<raisim::InertialMeasurementUnit>("imu_joint");

    for (int i=0; i<2000000; i++) {

//      std::cin.ignore();
//        imu->getAngularVelocity()


      go1->getState(gc,gv);


//      anymal_->getState(gc, gv);
      raisim::Vec<4> quat;
      raisim::Mat<3,3> rot;
      quat[0] = gc[3]; quat[1] = gc[4]; quat[2] = gc[5]; quat[3] = gc[6];
      quat /= quat.norm();
      raisim::quatToRotMat(quat, rot);
//      auto nn =go1->getCollisionBodies();
//      int cnt = 0;
//      for(auto n : nn )
//      {
//
//          std::cout<< cnt ++ << "   " << n.getMaterial() << std::endl;
//      }
//      std::cout<<world.getMaterialPairProperties();
//      std::cout<< go1->getCollisionBody("FL_FOOT/0").getMaterial();
      bodyLinearVelocity = rot.e().transpose() * gv.segment(0, 3);
      bodyAngularVelocity = rot.e().transpose() * gv.segment(3, 3);
      Eigen::Vector3d eule;
      rot2euler(rot, eule);
//      std::cout<<"euler " << bodyAngularVelocity<<std::endl;
//      std::cout << "body angular "<< bodyAngularVelocity[2] << std::endl;
//      std::cout << gv.segment(3,3) << std::endl;
//      raisim::Vec<4> qu{1+double(i)/500 , 0 , 0, 0+ double(i) / 500 };
//    go1->setBaseOrientation(qu);
//      Eigen::VectorXd tmp1(12), tmp2(12);
//      tmp1 = gc.tail(12);
//      tmp2 = gv.tail(12);
//      for(auto i=0; i<12; i++){
//          ob_[2*i] = tmp1[i];
//          ob_[2 * i+1] = tmp2[i];
//      }
//      for(auto i=0;i <12; i++)
//      {
//          swap(ob_, i, 12 +i);
//      }

      raisim::Vec<3> vel_temp;
//      go1->getFrameVelocity("ROOT", vel_temp);
//      for(auto i =0;i <2; i++)
//      {
//      acc = (vel_temp - vel) / 0.01;
      vel = vel_temp;
//          acc[i] = vel_temp
//      }
//      std::cout << "vel \n" << vel  << std::endl << "acc\n " << acc << std::endl;
      usleep(10000);
      angle_generator(position, i, 40, 0.35);
      jointNominalConfig.tail(12) = position;
//      for(auto n: go1->getContacts())
//      {
//          std::cout << n.getPairContactIndexInPairObject()<< std::endl;
//      }
//        for(auto obj:world.getObjList())
//        {
//            std::cout<<obj->getName()<<std::endl;
//        }
//        world.getContactProblem()

      Eigen::Quaterniond quata(gc[3], gc[4], gc[5], gc[6]);
      Eigen::Vector3d angle = ToEulerAngles(quata);
      Eigen::Matrix3d root(quata);

      bodyLinearVelocity = root.transpose() * gv.segment(0, 3);
      bodyAngularVelocity = root.transpose() * gv.segment(3, 3);
      if (abs(angle[0]) >abs(ma[0]) ) ma[0] = abs(angle[0]);
      if (abs(angle[1]) >abs(ma[1]) ) ma[1] = abs(angle[1]);
      if (abs(angle[2]) >abs(ma[2]) ) ma[2] = abs(angle[2]);
      raisim::Vec<3> angvel;
      go1->getAngularVelocity(roott, angvel);
//      std::cout<<quat << std::endl;
//      std::cout<< "angle" << b/odyAngularVelocity << std::endl;
//        std::cout<<"GET " << angvel << std::endl;
//        go1->printOutFrameNamesInOrder();
        go1->getFrameAcceleration("floating_base", angvel);
//        std::cout<<angle << std::endl;
//        std::cout <<"gravity " << world.getGravity();

//        std::cout<<"imu ang " << imu->getAngularVelocity();
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
//    jointNominalConfig[8] =  - 0.01 * i ;
go1->setPdTarget(jointNominalConfig, jointVelocityTarget);

//        for(auto& contact : go1->getContacts())
//        {
////            if(contact.getPairContactIndexInPairObject() != raisim::BodyType::STATIC)
////            {
////                world.getObject(contact.getPairObjectIndex())->getContacts();
////            }
//        }
//
    server.integrateWorldThreadSafe();

  }

  server.killServer();
}
