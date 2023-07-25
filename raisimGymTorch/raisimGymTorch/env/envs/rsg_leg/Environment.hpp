#pragma once
#include <stdlib.h>
#include <set>
#include <iostream>
#include "../../RaisimGymEnv.hpp"
#include <algorithm>
namespace raisim{

class ENVIRONMENT: public RaisimGymEnv{
public:
    explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable):
            RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable){
        world_ = std::make_unique<raisim::World>();

        leg_ = world_->addArticulatedSystem(resourceDir_ + "do_legs/do_legs.urdf") ; //
        leg_->setName("leg");
        leg_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
        world_ -> addGround();

        gcDim_ = leg_ -> getGeneralizedCoordinateDim();
        gvDim_ = leg_ -> getDOF();
//        nJoints_ = gvDim_ - 6;//
        nJoints_ = gcDim_;
        gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
        gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
        pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_);pTarget_.setZero(nJoints_);
        no_collision_.push_back(1);
        no_collision_.push_back(2);
        no_collision_.push_back(3);
//        no_collision_.push_back();

        gc_init_ << 0, 0, 0, 0, 0; //


        Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
        jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(100.0);
        jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(1);
        leg_ ->setPdGains(jointPgain, jointDgain);
        leg_ ->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

        obDim_ = 10;//
        actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
        obDouble_.setZero(obDim_);


        actionMean_ = gc_init_.tail(nJoints_);
        double action_std;
        READ_YAML(double, action_std, cfg_["action_std"]);
        actionStd_.setConstant(action_std);

        rewards_.initializeFromConfigurationFile(cfg["reward"]);


//        footIndices_.insert(leg_ ->getBodyIdx("shank")); // set for foot;
//        footIndices_.insert(leg_ ->getBodyIdx("knee")); // set for foot;
//        footIndices_.insert(leg_ ->getBodyIdx("thigh")); // set for foot;
//        footIndices_.insert(leg_ ->getBodyIdx("body")); // set for foot;
//        footIndices_.insert(leg_ ->getBodyIdx("lk5")); // set for foot;


        if(visualizable_)
        {
            server_ = std::make_unique<raisim::RaisimServer>(world_.get());
            server_ ->launchServer();
            server_ ->focusOn(leg_);
        }
    }



    void init() final {}

    void reset() final {
        leg_ ->setState(gc_init_, gv_init_);
        updateObservation();
    }

    float step(const Eigen::Ref<EigenVec>& action) final{
//        std::cout<< "action:\n" << action << std::endl;
        pTarget12_ = action.cast<double> (); // target for p ,12 is because of the 12 joints
        pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
        pTarget12_ += actionMean_;
        pTarget_.tail(nJoints_) = pTarget12_;
//        pTarget_[1] = 0;
//        std::cout<<pTarget_  << std::endl;
        leg_ ->setPdTarget(pTarget_, vTarget_);
        // note to excute the step command
        // the pd is just the controller to fit the position to the robot
        for(int i=0; i<int(control_dt_ / simulation_dt_ + 1e-10); i++)
        {
            if(server_) server_ -> lockVisualizationServerMutex();
            world_->integrate();
            if(server_) server_ -> unlockVisualizationServerMutex();
        }
        updateObservation();

//        rewards_.record("torque", leg_->getGeneralizedForce().squaredNorm());
//        rewards_.record("forwardVel", std::min(3.0, bodyLinearVel_[0]));
        rewards_.record("position", -bodyLinearPos_[1]); //
//        std::cout<< "height : " << std::endl<< -bodyLinearPos_[1] << std::endl;
        return rewards_.sum();
    }

    void updateObservation() {
        leg_ ->getState(gc_, gv_);
//        raisim::Vec<4> quat;
//        raisim::Mat<3, 3> rot;
//        quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
//        raisim::quatToRotMat(quat, rot);
//        bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
//        bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
//        std::cout << "data: " << leg_ ->getGeneralizedVelocity().data() << std::endl;


//        obDouble_ << gc_[2], // body height // 1
//                    rot.e().row(2).transpose(), // body orientation // 3
//                    gc_.tail(12), // joint angles // 12
//                    bodyLinearVel_, bodyAngularVel_, // body vel // 6
//                    gv_.tail(12); // 12
        bodyLinearVel_ =  gv_;
        bodyLinearPos_ =  gc_;
        obDouble_ << gc_.tail(5), gv_.tail(5);
    }

    void observe(Eigen::Ref<EigenVec> ob) final{
        ob = obDouble_.cast<float>( );
    }

    bool isTerminalState(float& terminalReward) final {
        terminalReward = float(terminalRewardCoeff_);
        for(auto& contact: leg_->getContacts())
        {
            std::vector<size_t>::iterator it = std::find(no_collision_.begin(), no_collision_.end(), contact.getlocalBodyIndex());
            if(it != no_collision_.end()) // if it != end ,means it was found
                return true;
        }

        terminalReward = 0.f;
        return false;
    }

    void curriculumUpdate()  {};


private:
    int gcDim_, gvDim_, nJoints_;
    bool visualizable_ = false;
    raisim::ArticulatedSystem* leg_;
    Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
    double terminalRewardCoeff_ = -10.;
    Eigen::VectorXd actionMean_, actionStd_, obDouble_;
    Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, bodyLinearPos_;
    std::set<size_t> footIndices_;
    std::vector<size_t> no_collision_;

};


}