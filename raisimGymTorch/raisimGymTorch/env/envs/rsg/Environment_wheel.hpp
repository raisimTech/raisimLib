//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
//#include "yaml.h"
#include "../../RaisimGymEnv.hpp"
#include <iostream>
#define PI 3.1415926

namespace raisim {

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

    class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {
    /// create world
    world_ = std::make_unique<raisim::World>();

      READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
      READ_YAML(bool, show_ref, cfg_["show_ref"]);
      if (show_ref == 0) show_ref = false;
      READ_YAML(double, angle_rate, cfg_["angle_rate"]);
      READ_YAML(bool, float_base, cfg_["float_base"]);
      READ_YAML(float,schedule_T, cfg_["schedule"]);
      READ_YAML(std::string, urdf_path, cfg_["urdf_path"]);
      READ_YAML(double, p_gain, cfg_["p_gain"]);
      READ_YAML(double, d_gain, cfg_["d_gain"]);
    anymal_ = world_->addArticulatedSystem(urdf_path);
//    skate = world_ ->addArticulatedSystem("/home/lr-2002/code/raisimLib/rsc/skate/skate.urdf");
    anymal_->setName("model");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround(0,"land");
//world_->addGround();
    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim(); // 19
    gvDim_ = anymal_->getDOF(); // 18
    nJoints_ = gvDim_ - 6;
//    skate_posi_init.setZero(9);
//    skate_vel_init.setZero(8);
//    skate_posi_init << 0, 0.15, 0.11, 0.707, 0., 0, 0.707, 0 , 0;
//    skate ->setGeneralizedCoordinate(skate_posi_init);
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    euler_angle.setZero();euler_angle_old.setZero();
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    angle_list.setZero(nJoints_);angle_list_for_work.setZero(nJoints_);
    gc_old.setZero(nJoints_) ; ref_old.setZero(nJoints_);
    acc_.setZero();//1
    ang_vel_.setZero();
//    skate_vel_.setZero(8);
//    skate_posi_.setZero(9);
//      init_position();
double aa =  double(49)/ 180 * PI , bb = double(49) /180*PI;
    double for_r = double(0) / 180 * PI ;
    double abss = double(8) / 180 * PI;
    gc_init_<< 0, 0, 0.31, 1.0, 0.0, 0.0, 0.0, 0.0,  aa, -2*aa + 2 *abss, 0.0, bb, -2*bb+ 2 *abss, 0.0,aa +for_r ,-2*aa -2 * for_r+ 2 *abss , 0.0, bb + for_r, -2*bb -2 * for_r+ 2 *abss;
//    gc_init_<< 0, 0, 0.37, 1.0, 0.0, 0.0, 0.0, 0.0,  0.5233, -1.046, 0.0,  0.5233, -1.046, 0.0, 0.523, -1.046, 0.0, 0.523, -1.046;
    init();

    obDim_ = 29;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);

    actionStd_.setConstant(action_std);

    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    footIndices_.insert(anymal_->getBodyIdx("FL_calf"));
    footIndices_.insert(anymal_->getBodyIdx("FR_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RL_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RR_calf"));

//      for(auto n : anymal_->getCollisionBodies())
//      {
//          auto name = n.colObj->name;
//          anymal_->getCollisionBody(name).setMaterial("steel");
//      }

//      skate->getCollisionBody("base/0").setMaterial("sandpaper");
//      skate->getCollisionBody("rotater_r/0").setMaterial("rubber");
//      skate->getCollisionBody("rotater_f/0").setMaterial("rubber");
      anymal_->getCollisionBody("FL_foot/0").setMaterial("rubber");
      anymal_->getCollisionBody("FR_foot/0").setMaterial("rubber");
      anymal_->getCollisionBody("RR_foot/0").setMaterial("rubber");
      anymal_->getCollisionBody("RL_foot/0").setMaterial("rubber");
//      world_->setMaterialPairProp("steel", "rubber", 0.8, 0.15, 0.001);
//      world_->setMaterialPairProp("rubber", "sandpaper", 0.99, 0.15, 0.001);
      world_->setMaterialPairProp("land", "rubber", 0.8, 0.1,0.001);
//      world_->setMaterialPairProp("sandpaper", "land", 0.4, 0.15,0.001);
//      world_->setMaterialPairProp("steel","land", 0.1, 0.05,0.001);

    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(anymal_);
    }
  }



  void init() final {
      Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
      jointPgain.setZero();
      jointPgain.tail(nJoints_).setConstant(p_gain);
      jointDgain.setZero();
      jointDgain.tail(nJoints_).setConstant(d_gain);
      anymal_->setPdGains(jointPgain, jointDgain);
      anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
      if(show_ref)
      {
          std::cout<<"using ref";
          anymal_1 = world_->addArticulatedSystem(urdf_path);
          anymal_1 ->setName("anymal_ref");
          anymal_1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
          anymal_1->setPdGains(jointPgain,jointDgain);
      }
  }

  void init_position(const Eigen::Ref<EigenVec>& posi) final
  {
      gc_init_ = posi.cast<double>();
      init();
  }

  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
//    skate->setState(skate_posi_init, skate_vel_init);
    rewards_.setZero();
    COUNT=0;
    limit_flag = false;
//    euler_angle_old.setZero();
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
//      std::cout<<"mt 19937 " << gen_() << std::endl;
      std::uniform_int_distribution<int> rnddd(1, 100);
      rnd_cnt = rnddd(gen_);
    if (show_ref)
    {
        Eigen::Vector3d po(3, 3 ,10);
        anymal_1->setBasePos(po);
        pTarget_.tail(nJoints_) = angle_list;
        anymal_1->setPdTarget(pTarget_, vTarget_);
    }
    if (float_base)
    {

        Eigen::Vector3d po(3, 3 ,10);
        anymal_->setBasePos(po);

    }

    COUNT ++;
    pTarget12_ = action.cast<double>();

    for (auto i = 0;i <12; i++)
        {
        if(abs( p_gain * (pTarget12_[i] - gc_[7 + i ]) + d_gain * (gv_[7+i])) > 50)
            {
            limit_flag = false;
            }
        }
    ori = pTarget12_[0];
    pTarget12_ = pTarget12_.tail(nJoints_);

    pTarget_.tail(nJoints_) = pTarget12_;

    anymal_->setPdTarget(pTarget_, vTarget_);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }



    /// apply random force
//    raisim::Vec<3> force_ = {gen_()  % 10, };
if (rnd_cnt <=2){
    std::uniform_real_distribution<double> force_dist(-30,30);
//    std::uniform_int_distribution<int> idx_dist(0, 13);
    raisim::Vec<3> force_ = {force_dist(gen_), force_dist(gen_), force_dist(gen_)};
//    std::cout << " force gened " << force_ << std::endl;
//    int iddx =
    anymal_->setExternalForce(0, force_);
//    anymal_->printOutMovableJointNamesInOrder();
}


    updateObservation();
    double rrr =0;
    rrr = abs(euler_angle[0]) + abs(euler_angle[1]) +abs(euler_angle[2]);
    rrr = rrr * 0.1;
    rrr += 0.1 * ( abs(ang_vel_[0]) + abs(ang_vel_[1]) +abs(ang_vel_[2]));
//    rrr += abs(gc_[0] - skate_posi_[0]) + abs(gc_[1] - (skate_posi_[1] - 0.15));
    bool accu = false;
//    std::cout << "body: " << bodyLinearVel_ << "\n  get_vel  " << line_vel_ << std:: endl ;
    rewards_.record("Stable",-rrr, accu);
    rewards_.record("Live", 1, accu);
//    std::cout << "minus is " << bodyLinearVel_[0]  << "    " << line_vel_[0] << std::endl;
//    rewards_.record("forwardVel",bodyLinearVel_[0], accu);
    rewards_.record("forwardVel",line_vel_[0], accu);
//    rewards_.record("height", 0.45- abs(gc_[2] - 0.45) - abs(gc_[0] ) - abs(gc_[1]) , accu);
//    rewards_.record("Mimic", (gc_.tail(12) - pTarget12_).norm(), accu);
//    rewards_.record("Wheel", euler_angle[2] * double(COUNT) / 400, accu);
    rewards_.record("Wheel", ang_vel_[2], accu);
//    rewards_.record("torque",anymal_->getGeneralizedForce().squaredNorm() );
    return rewards_.sum();
  }

  void updateObservation() {
//  raisim::Vec<3> acc_w;

//  std::cout<< "acc? " <<anymal_->getFrameAcceleration("base",acc_w);
    anymal_->getState(gc_, gv_);
//    skate -> getState(skate_posi_, skate_vel_);

    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    quat /= quat.norm();
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    anymal_->getAngularVelocity(anymal_->getBodyIdx("base"), ang_vel_);
    anymal_->getFrameVelocity(anymal_->getBodyIdx("base"), line_vel_);
    Eigen::Quaterniond qua(gc_[3], gc_[4], gc_[5], gc_[6]);
    euler_angle = ToEulerAngles(qua);
    Eigen::VectorXd gcc = gc_.tail(12);
    Eigen::VectorXd gvv = gv_.tail(12);
    Eigen::VectorXd c_v(24);
    c_v.setZero(24);
    for(auto i = 0; i <12 ; i++ )
    {
        c_v[i*2] = gcc[i];
        c_v[i*2 + 1] = gvv[i];
    }

    obDouble_ <<
        euler_angle[0],
       euler_angle[1],// quaternion
//        ori,
        ang_vel_[0],
        ang_vel_[1],
        ang_vel_[2],
//        line_vel_[0],
//        line_vel_[1],
       c_v;
//       gcc;

  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);
    bool accu = true;
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
      {// if there is any contact body was not in the footIndices the over
          rewards_.record("Live", terminalReward, accu);
//           std::cout<<"foot done " << std::endl;
          return true;}

//      if(gc_[2] - gc_init_[2] > 0.3){
////      std::cout<<"z done" << std::endl;
//         rewards_.record("Live", terminalReward, accu);
//       return true;
//       }

      if(fmin(abs(euler_angle[1]), abs(euler_angle[1] + 2 * PI)) > 0.17)
      {
//      std::cout<<"y angle done" <<"  " << euler_angle[1]<< std::endl;
          rewards_.record("Live", terminalReward, accu);

        return true;
       }
      if(fmin(abs(euler_angle[0]), abs(euler_angle[0] + 2 * PI)) > 0.4)
      {
//      std::cout<<"x angle done " << euler_angle[0] << std::endl;
          rewards_.record("Live", terminalReward, accu);

            return true;
        }
      if(limit_flag)
          {
//          std::cout<<"limit done" << std::endl;
          rewards_.record("Live", terminalReward, accu);
          return true;
          }
//        if(abs(gc_[0] - skate_posi_[0]) >0.2)
//        {
//            return true;
//        }
//        if(abs(gc_[1] - (skate_posi_[1] - 0.15)) >0.1)
//        {
//            return true;
//        }

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
 bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_, *anymal_1;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -60.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Vec<3> acc_, ang_vel_, line_vel_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
  double p_gain,d_gain;
  std::string urdf_path;
  std::vector<raisim::Vec<2>>  join_limit;
  int COUNT = 0;
  float schedule_T;
  bool show_ref= true;
  double ori = 1.0;
  double limit_flag = false;
  double action_std, angle_rate;
  Eigen::VectorXd angle_list, angle_list_for_work;
  Eigen::VectorXd gc_old, ref_old;
  Eigen::Vector3d euler_angle, euler_angle_old;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  bool float_base;
//  Eigen::VectorXd  skate_vel_, skate_posi_,skate_posi_init, skate_vel_init;
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
  int rnd_cnt = 0;
//  static std::uniform_real_distribution<double> dist_(-10,10);
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}
