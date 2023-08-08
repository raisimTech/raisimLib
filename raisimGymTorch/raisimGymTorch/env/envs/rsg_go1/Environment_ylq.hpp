//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
//#include "yaml.h"
#include "../../RaisimGymEnv.hpp"
#define PI 3.1415926

double cal(double low, double upp, double now){
    return (now + 1)/2  * (upp- low) + low;
}

void map_from_origin_to_limit(std::vector<raisim::Vec<2>>  join, Eigen::VectorXd& limit_list, Eigen::VectorXd& gen_list)
{
//    Eigen::VectorXd  limit_list;
//    join = join.tail(12);
    limit_list.setZero(join.size() - 6);
//    std::cout<< "!------------" << gen_list.size()<< std::endl;
    int cnt = 0;
    int jp = 6;
    int kk = 0;
    for( auto i : join)
    {
        if(kk < jp){
            kk++;
            continue;
        }
//        std::cout << i[1] << " " << i[0]<< std::endl;
        limit_list[cnt] = (cal(i[0] , i[1], gen_list[cnt]));
        cnt++;
    }
}

//void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
//{
////    std::cout<< "size is "<<  angle_list.size() << std::endl;
//    double base1= 0.82, base3=0.0;
//    double base2 = -2 * base1;
//    double ang = abs(sin( float(idx) / T  * PI)) * rate;
//
//    int idx_base = 0;
////    std::cout<<idx_base+0 << " " << idx_base + 11 << std::endl;
////    jointNominalConfig <<  0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    if ( (int(idx/T) % 2 ) == 1)
//    {
//
//        angle_list[idx_base+1] = ang + base1;
//        angle_list[idx_base+2] = -2 * ang + base2;
//
//        angle_list[idx_base+4] = base1;
//        angle_list[idx_base+5] = base2;
//
//        angle_list[idx_base+7] = base1;
//        angle_list[idx_base+8] = base2;
//        angle_list[idx_base+10] = ang + base1;
//        angle_list[idx_base+11] = - 2 * ang + base2;
//    }
//    else
//    {
//
//        angle_list[idx_base+1] = base1;
//        angle_list[idx_base+2] = base2;
//
//        angle_list[idx_base+10] = base1;
//        angle_list[idx_base+11] = base2;
//        angle_list[idx_base+4] = ang + base1;
//        angle_list[idx_base+5] = -2 * ang + base2;
//
//        angle_list[idx_base+7] = ang + base1;
//        angle_list[idx_base+8] = -2 * ang + base2;
//    }
//    angle_list[idx_base+0] = base3;
//    angle_list[idx_base+3] = base3;
//    angle_list[idx_base+6] = base3;
//    angle_list[idx_base+9] = base3;
//
//}
void angle_generator(Eigen::VectorXd& angle_list, int idx, float T, float rate=1.f)
{
    // todo ang2, rate, ang
//    std::cout<< "size is "<<  angle_list.size() << std::endl;
    double base1= 0.8, base3=0.0;
    double base2 = -2 * base1;
    double ang = -abs(sin( float(idx) / T  * PI)) * rate;
    double ang2 = -0.15 * ang;
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

void angle_mulit(Eigen::VectorXd& angle, Eigen::VectorXd& idx, float rate)
{
    for(auto i : idx)
    {
        angle[int(i)] *= rate;
    }
}
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
    /// add objects
    anymal_ = world_->addArticulatedSystem(resourceDir_+"/a1/urdf/a1.urdf");
    anymal_->setName("dog");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);
    angle_list.setZero(nJoints_);angle_list_for_work.setZero(nJoints_);
    gc_old.setZero(nJoints_) ; ref_old.setZero(nJoints_);
    /// this is nominal configuration of anymal
//    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    gc_init_ << 3, 3, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
//    anymal_1->setGeneralizedCoordinate(gc_init_);
      gc_init_<< 0, 0, 0.30, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;

//  jointNominalConfig.tail(12).setZero();
      Eigen::VectorXd tmp(12);
      tmp.setZero();
      angle_generator(tmp, 0, 80.f);
      gc_init_.tail(12) = tmp;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(120.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(3);
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));


    if(show_ref)
    {
        std::cout<<"using ref";
        anymal_1 = world_->addArticulatedSystem(resourceDir_+"/anymal_c/urdf/anymal.urdf");
        anymal_1 ->setName("anymal_ref");
        anymal_1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
        anymal_1->setPdGains(jointPgain,jointDgain);
    }

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);

//      READ_YAML();
    actionStd_.setConstant(action_std);

    join_limit = anymal_->getJointLimits();
    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
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

  void init() final { }

  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
    rewards_.setZero();
    COUNT=0;
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    COUNT ++;
//    std::cout<<COUNT<<std::endl;
    angle_generator(angle_list, COUNT, schedule_T, angle_rate);
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


//        raisim::Vec<4> quat;
//        raisim::Mat<3,3> rot;
//        quat[0] = 0; quat[1] = 0; quat[2] = 0; quat[3] = 0;
//        raisim::quatToRotMat(quat, rot);
//        anymal_->setBaseOrientation(rot);
//        raisim::Vec<4> quat(4);
//        quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
//        anymal_->setBaseOrientation(quat);
//        pTarget_.tail(nJoints_) = angle_list;
//        anymal_->setPdTarget(pTarget_, vTarget_);
    }
    //    angle_list *= 0.3;
//    angle_list.setZero();
    Eigen::VectorXd idx1(8), idx2(4);
    idx1.setZero();idx2.setZero();
    idx1<<1, 2, 4, 5, 7,8,10,11;
    idx2<<0, 3, 6, 9;

    pTarget12_ = action.cast<double>();
    Eigen::VectorXd ttmp =action.cast<double>();
      map_from_origin_to_limit(join_limit, pTarget12_,ttmp);
    //    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
//    pTarget12_ += actionMean_;
//    pTarget12_ = angle_list;


    angle_mulit(pTarget12_, idx1, action_std);
    angle_mulit(pTarget12_, idx2, action_std);


    angle_list_for_work = angle_list * for_work_rate;




    pTarget12_ = angle_list_for_work + pTarget12_;
//    pTarget12_[11] = pTarget12_[10] *-2;
//    pTarget12_[8] = pTarget12_[7] * -2;
//    pTarget12_[1] = -pTarget12_[10];
//    pTarget12_[2] = -pTarget12_[11];
//    pTarget12_[4] = -pTarget12_[7];
//    pTarget12_[5] = -pTarget12_[8];
//    pTarget12_[9] = -0.03;
//    pTarget12_[0] = -pTarget12_[9];
//    pTarget12_[6] = 0.03;
//    pTarget12_[3] = -pTarget12_[6];
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
//    rrr *= float(COUNT )/ schedule_T);
//      for(int i=0; i<=1; i++)
//      {
//          rrr += abs(gc_[i] - gc_init_[i]) * stable_reward_rate ;
//      }
//      rrr += abs(gc_[5] - gc_init_[5]) * 2 * stable_reward_rate;
//    if(COUNT != 0)
//    {
//        rrr += (gc_ - gc_init_).norm() * reference_rate *0.1;
//    }


//    rrr +=(gc_init_.head(7) - gc_.head(7)).norm()   * stable_reward_rate;
    rrr += (gc_.tail(12) - angle_list).norm() * reference_rate;
//    rrr += abs(bodyLinearVel_[1]);
//      rrr += abs(bodyLinearVel_[2]);
//      rrr += abs(bodyLinearVel_[0]);

//      rrr += abs(gc_[2] - gc_init_[2]) ;

//    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());
//    rewards_.record("forwardVel", std::mi);
    rewards_.record("Stable", 1 - rrr, false);
    rewards_.record("forwardVel", bodyLinearVel_[0], false);

    gc_old = gc_.tail(12);
    ref_old = angle_list;
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
//    std::cout<< rot.e().row(2).transpose().size() << "   "  <<bodyLinearVel_.size()  << "  " << bodyAngularVel_.size() << std::endl;
    obDouble_ << double(COUNT)/schedule_T,
        rot.e().row(2).transpose(), /// body orientation
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gc_.tail(12); /// joint angles
        gv_.tail(12); /// joint velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

//    / if the contact body is not feet
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
      {// if there is any contact body was not in the footIndices the over
//          std::cout<<"terminate "<<std::endl;
          rewards_.record("Stable", -10, false);
          return true;
      }
//    if(abs(gc_[2]-gc_init_[2]) >0.3 )
//    {
//        return true;
//    }
//    if(abs(gc_[9]) > 1.8 || abs(gc_[12]) > 1.8 || abs(gc_[15]) > 1.8 || abs(gc_[18]) > 1.8)
//        return true;

//    if(abs(gc_[9]) > 1.8 || abs(gc_[12]) > 1.8 || abs(gc_[15]) > 1.8 || abs(gc_[18]) > 1.8 || )
//        return true;

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
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
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
