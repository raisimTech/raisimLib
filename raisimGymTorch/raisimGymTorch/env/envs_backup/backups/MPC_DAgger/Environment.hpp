//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"


namespace raisim {



class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    anymal_ = world_->addArticulatedSystem(resourceDir_+"/a1/urdf/a1.urdf");
    anymal_->setName("a1");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();
    
    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_); tTarget12_.setZero(nJoints_);
    additional_torques.setZero(gvDim_);
    gc_init_ << 0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8;
    
    /// set pd gains
    jointPgain_init.setZero(gvDim_); 
    jointPgain_init.tail(nJoints_).setConstant(100.0);
    jointDgain_init.setZero(gvDim_);
    jointDgain_init.tail(nJoints_).setConstant(2.0);
    jointDgain_init(6) = 1.0;
    jointDgain_init(9) = 1.0;
    jointDgain_init(12) = 1.0;
    jointDgain_init(15) = 1.0;

    jointPgain = jointPgain_init;
    jointDgain = jointDgain_init;
    
    anymal_->setPdGains(jointPgain_init, jointDgain_init);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));  
    anymal_->setJointDamping(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS. set action and observations
    obDim_ = 26;
    actionDim_ = nJoints_*4; 
    actionMean_.setZero(nJoints_); 
    actionStd_.setZero(nJoints_);
    actionStd_torque.setZero(nJoints_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.3);
    actionStd_torque.setConstant(5);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.insert(anymal_->getBodyIdx("FR_calf"));
    footIndices_.insert(anymal_->getBodyIdx("FL_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RR_calf"));
    footIndices_.insert(anymal_->getBodyIdx("RL_calf"));

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(anymal_);
    }
  }

  void init() final {    
  }
  void setSeed(int seed) final {
  }
  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
    ///////////////////////////// my stabilization code /////////////////////////////////
    pTarget_.head(7) = Eigen::VectorXd::Zero(7);
    pTarget_.tail(nJoints_) = gc_init_.tail(nJoints_);
    vTarget_.setZero(gvDim_);
    anymal_->setPdGains(jointPgain_init, jointDgain_init);
    anymal_->setPdTarget(pTarget_,vTarget_);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
    anymal_->setJointDamping(Eigen::VectorXd::Zero(gvDim_));

    for(int i=0;i<500;i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }
    /////////////////////////////////////////////////////////////////////////////////////
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    // position action
    pTarget12_ = action.head(nJoints_).cast<double>();
    // pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    // pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    // torque action
    // tTarget12_ = action.tail(nJoints_).cast<double>();
    // tTarget12_ = tTarget12_.cwiseProduct(actionStd_torque);
    // additional_torques.tail(nJoints_) = tTarget12_;
    tTarget12_ = action(Eigen::seq(12,23)).cast<double>();
    additional_torques.tail(nJoints_) = tTarget12_;

    // kp action
    jointPgain.tail(nJoints_) = action(Eigen::seq(24,35)).cast<double>();
    jointDgain.tail(nJoints_) = action(Eigen::seq(36,47)).cast<double>();
    

    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setPdTarget(pTarget_, vTarget_);
    anymal_->setGeneralizedForce(additional_torques);

    if(server_) server_->lockVisualizationServerMutex();
    world_->integrate();
    if(server_) server_->unlockVisualizationServerMutex();

    // for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
    //   if(server_) server_->lockVisualizationServerMutex();
    //   world_->integrate();
    //   if(server_) server_->unlockVisualizationServerMutex();
    // }
    
    updateObservation();
    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());
    rewards_.record("forwardVel",exp(-(0.6-bodyLinearVel_[0])*(0.6-bodyLinearVel_[0])));

    return rewards_.sum();
  }

  void updateObservation() {
    anymal_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    footContacts.setZero(4);
    for(auto& contact: anymal_->getContacts())
    {
      if(contact.skip()) continue; //internal contact
      int idx = 0;
      for(auto u : footIndices_)
      {
        if(u == contact.getlocalBodyIndex()) break;
        idx++;
      }
      if(idx<4) footContacts[idx] = 1.0;
    }

    obDouble_ <<
        gc_(Eigen::seq(3,6)),
        footContacts,
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_; /// body linear&angular velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
        return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, 
        vTarget_,additional_torques, tTarget12_, footContacts, jointPgain, jointDgain,
        jointPgain_init, jointDgain_init;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, actionStd_torque,obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}
