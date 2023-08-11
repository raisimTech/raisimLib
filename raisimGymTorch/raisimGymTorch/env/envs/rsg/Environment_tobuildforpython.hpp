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
      READ_YAML(double, stable_reward_rate, cfg_["stable"]);
      READ_YAML(double, reference_rate, cfg_["reference"]);
      READ_YAML(double, for_work_rate, cfg_["for_work"]);
      READ_YAML(bool, float_base, cfg_["float_base"]);
      READ_YAML(float,schedule_T, cfg_["schedule"]);
      READ_YAML(std::string, urdf_path, cfg_["urdf_path"]);
      READ_YAML(double, p_gain, cfg_["p_gain"]);
      READ_YAML(double, d_gain, cfg_["d_gain"]);
    anymal_ = world_->addArticulatedSystem(urdf_path);
    anymal_->setName("model");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim(); // 19
    gvDim_ = anymal_->getDOF(); // 18
    nJoints_ = gvDim_ - 6;

    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    euler_angle.setZero();
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    angle_list.setZero(nJoints_);angle_list_for_work.setZero(nJoints_);
    gc_old.setZero(nJoints_) ; ref_old.setZero(nJoints_);
    acc_.setZero();//1

//      init_position();
    gc_init_<< 0, 0, 0.37, 1.0, 0.0, 0.0, 0.0, 0.0,  0.5233, -1.046, 0.0,  0.5233, -1.046, 0.0, 0.523, -1.046, 0.0, 0.523, -1.046;
//      init_position(gc_init_);
    init();

    obDim_ = 31;
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

    /// visualize if it is the first environment
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
      jointPgain.tail(nJoints_)[2] = 300;
      jointPgain.tail(nJoints_)[5] = 300;
      jointPgain.tail(nJoints_)[8] = 300;
      jointPgain.tail(nJoints_)[11] = 300;
      jointDgain.setZero();
      jointDgain.tail(nJoints_).setConstant(d_gain);
      jointDgain.tail(nJoints_)[2] = 15;
      jointDgain.tail(nJoints_)[5] = 15;
      jointDgain.tail(nJoints_)[8] = 15;
      jointDgain.tail(nJoints_)[11] = 15;
      anymal_->setPdGains(jointPgain, jointDgain);
//      std::cout<<jointPgain;
      anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
      anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
      if(show_ref)
      {
          std::cout<<"using ref";
          anymal_1 = world_->addArticulatedSystem(urdf_path);
          anymal_1 ->setName("anymal_ref");
          anymal_1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
          anymal_1->setPdGains(jointPgain,jointDgain);
      }
//      std::cout<<"position inited\n";
  }

  void init_position(const Eigen::Ref<EigenVec>& posi) final
  {
      gc_init_ = posi.cast<double>();
      init();
  }

  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
    rewards_.setZero();
    COUNT=0;
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
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

    pTarget12_ = action.cast<double>();
    pTarget_.tail(nJoints_) = pTarget12_;

    anymal_->setPdTarget(pTarget_, vTarget_);

    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }
    updateObservation();
    double rrr =0;
//    for(int i=4; i<=6 ; i++)
//    {
//        rrr += abs(gc_[i] - gc_init_[i]) * stable_reward_rate ;
//    }
//    for(int i =0;i<=2; i++)rrr += abs(bodyAngularVel_[i]);
//std::cout<<euler_angle;
    for(int i=0;i<=2;i++) rrr += abs(euler_angle[i]) * stable_reward_rate;

    rrr += (gc_.tail(12) - pTarget12_).norm() * reference_rate;
    rewards_.record("Stable", 1 - rrr, true);
//    rewards_.record("forwardVel", bodyLinearVel_[0], false);

//    gc_old = gc_.tail(12);
//    ref_old = angle_list;
    return rewards_.sum();
  }

//  auto get_orientation()
//  {
//
//      raisim::Vec<4> quat;
//
//      quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
//      return quat.e();
//  }
//
//  auto get_body_linear_velocity(){
//      return bodyLinearVel_;
//  }
//  auto get_body_angular_velocity()
//  {
//      return bodyAngularVel_;
//  }
//
//  auto get_joint_coordinate()
//  {
//      return gc_;
//  }
//
//  auto get_joint_velocity()
//  {
//      return gv_;
//  }
  void updateObservation() {
    anymal_->getState(gc_, gv_);
//    std::string root_name = "ROOT";
//    anymal_->getFrameAcceleration(root_name , acc_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    quat /= quat.norm();
//    std::cout << quat << std::endl;
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    Eigen::VectorXd gcc = gc_.tail(12);
    Eigen::VectorXd gvv = gv_.tail(12);
//    std::cout << "gcc_ : \n " << gcc << std::endl << "gvv : \n" << gvv << std::endl;
    Eigen::VectorXd c_v(24);
    c_v.setZero(24);
    for(auto i = 0; i <12 ; i++ )
    {
        c_v[i*2] = gcc[i];
        c_v[i*2 + 1] = gvv[i];
    }
//    std::cout << "quat" << quat << "\n body angular vel" <<  bodyAngularVel_ << "\n  acc " << acc_   << "\nc_v "<< c_v << std::endl;
    obDouble_ << quat[0], quat[1], quat[2], quat[3],   // quaternion
        bodyAngularVel_[0], // ras/s
            bodyAngularVel_[1], // ras/s
            bodyAngularVel_[2], // ras/s
//        acc_[0],  // m/s^2
//            acc_[1],  // m/s^2
//            acc_[2],  // m/s^2 # todo recall acc
        c_v;
//    std::cout<< "obdouble \n" << obDouble_ << std::endl;

  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    Eigen::Quaterniond quat(gc_[3], gc_[4], gc_[5], gc_[6]);
//    std::cout<<"quat" << quat.w()  << " " <<quat.x() << " "<< quat.y() << " " << quat.z() <<std::endl;

    euler_angle = ToEulerAngles(quat);
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
      {// if there is any contact body was not in the footIndices the over
          rewards_.record("Stable", -10, true);
          return true;}
      if(abs(gc_[2] - gc_init_[2]) > 0.3){ return true;}
      if(fmin(abs(euler_angle[1]), abs(euler_angle[1] + 2 * PI)) > 0.1){ return true;}
      if(fmin(abs(euler_angle[0]), abs(euler_angle[0] + 2 * PI)) > 0.1){ return true;}
      if(fmin(abs(euler_angle[2]), abs(euler_angle[2] + 2 * PI)) > 0.1){ return true;}
      if(abs(gc_.tail(12)[0]) >0.17) return true;
      if(abs(gc_.tail(12)[3]) >0.17) return true;
      if(abs(gc_.tail(12)[6]) >0.17) return true;
      if(abs(gc_.tail(12)[9]) >0.17) return true;
    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
 bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_, *anymal_1;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Vec<3> acc_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
  double p_gain,d_gain;
  std::string urdf_path;
  std::vector<raisim::Vec<2>>  join_limit;
  int COUNT = 0;
  float schedule_T;
  bool show_ref= true;
  double action_std, angle_rate, reference_rate, stable_reward_rate,for_work_rate;
  Eigen::VectorXd angle_list, angle_list_for_work;
  Eigen::VectorXd gc_old, ref_old;
  Eigen::Vector3d euler_angle;
  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  bool float_base;
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}
