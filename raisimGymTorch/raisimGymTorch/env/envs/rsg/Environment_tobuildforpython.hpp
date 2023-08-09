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
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF(); // 18
    nJoints_ = gvDim_ - 6;

    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    angle_list.setZero(nJoints_);angle_list_for_work.setZero(nJoints_);
    gc_old.setZero(nJoints_) ; ref_old.setZero(nJoints_);
    acc_.setZero();//1

//      init_position();
    gc_init_<< 0, 0, 0.33, 1.0, 0.0, 0.0, 0.0, 0.0, 0.4, -0.8529411764705883, 0.0, 0.4, -0.8529411764705883, 0.0, 0.4, -0.8529411764705883, 0.0, 0.4, -0.8529411764705883;
//      init_position(gc_init_);
    init();

    obDim_ = 34;
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
      jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(p_gain);
      jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(d_gain);
      anymal_->setPdGains(jointPgain, jointDgain);
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
      std::cout<<"position inited\n";
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
    for(int i=4; i<=6 ; i++)
    {
        rrr += abs(gc_[i] - gc_init_[i]) * stable_reward_rate ;
    }
//    rrr += (gc_.tail(12) - angle_list).norm() * reference_rate;
    rewards_.record("Stable", 1 - rrr, false);
    rewards_.record("forwardVel", bodyLinearVel_[0], false);

    gc_old = gc_.tail(12);
    ref_old = angle_list;
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
    std::string root_name = "ROOT";
    anymal_->getFrameAcceleration(root_name , acc_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    Eigen::VectorXd gcc = gc_.tail(12);
    Eigen::VectorXd gvv = gv_.tail(12);
    Eigen::VectorXd c_v(24);
    for(auto i = 0; i <12 ; i++ )
    {
        c_v <<  gcc[i], gvv[i];
    }
    obDouble_ << quat,   // quaternion
        bodyAngularVel_, // ras/s
        acc_,  // m/s^2
//        gc_.tail(12), // rad/s
//        gv_.tail(12); // rad/s^2
        c_v;

  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    Eigen::Quaternion<double> quat(gc_[3], gc_[4], gc_[5], gc_[6]);
    Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0,1,2);
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
      {// if there is any contact body was not in the footIndices the over
          rewards_.record("Stable", -10, false);
          return true;}
      if(abs(gc_[2] - gc_init_[2]) < 0.5) return false;
      if(fmin(abs(euler[1]), abs(euler[1] - 360.0)) > 0.175) return false;
      if(fmin(abs(euler[0]), abs(euler[0] - 360.0)) > 0.175) return false;
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
  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  bool float_base;
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}
