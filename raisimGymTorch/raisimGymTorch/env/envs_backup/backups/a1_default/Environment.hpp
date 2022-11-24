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

    ///////////////////////////////////////////////////////////////////////////////////////////
    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of anymal
    gc_init_ << 0.0, 0.0, 0.27, 1.0, 0.0, 0.0, 0.0, 
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8, 
                0.0, 0.9, -1.8;
    

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(100.0);
    // jointDgain.setZero(); 
    jointDgain.setZero();
    jointDgain.tail(nJoints_).setConstant(2.0);
    jointDgain(6) = 1.0;
    jointDgain(9) = 1.0;
    jointDgain(12) = 1.0;
    jointDgain(15) = 1.0;
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_)); 


    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.3);

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
    updateObservation();
    
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    anymal_->setPdTarget(pTarget_, vTarget_);
    // world_->integrate();
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }
    
    updateObservation();
    ///////////////////// my comment ////////////////////////////////////////
    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());
    rewards_.record("forwardVel", std::min(4.0, bodyLinearVel_[0]));
    // rewards_.record("forwardVel",exp(-fabs(bodyLinearVel_[0]-0.6)));
    //////////////////////////////////////////////////////////////////////
    
    //////////////////////////////////////////////my reward////////////////////////////////////////////////////////////////////

    // //Get rotation matrix
    // raisim::Vec<4> base_orient;
    // anymal_->getBaseOrientation(base_orient);
    // raisim::Mat<3,3> rot_mat;
    // raisim::quatToRotMat(base_orient, rot_mat);
    // // 1. Compute r_torso
    // Eigen::VectorXd gv_my,gc_my;
    // anymal_->getState(gc_my,gv_my);
    // Eigen::Vector3d lin_vel_my = rot_mat.e().transpose() * gv_my.segment(0,3);// hat(v)_xy*v_xy
    // Eigen::Vector3d angle_vel_my = rot_mat.e().transpose() * gv_my.segment(3,3);
    // double r1 = std::min(lin_vel_my[0],0.5);
    // double m = 0.5;
    // double w = atanh(sqrt(0.95))/m;
    // double c_prec = tanh((angle_vel_my[2]*w) * (angle_vel_my[2] *w));
    // double k = 1-c_prec;
    // double r_torso = std::min(k*r1,r1);
    // rewards_.record("r_torso",r_torso);
    // // 2. Foot reward
    // raisim::Vec<3> footVel1,footVel2,footVel3,footVel4;
    // auto footFrameIndex1 = anymal_->getFrameIdxByName("LF_ADAPTER_TO_FOOT"); 
    // auto footFrameIndex2 = anymal_->getFrameIdxByName("RF_ADAPTER_TO_FOOT"); 
    // auto footFrameIndex3 = anymal_->getFrameIdxByName("LH_ADAPTER_TO_FOOT"); 
    // auto footFrameIndex4 = anymal_->getFrameIdxByName("RH_ADAPTER_TO_FOOT");
    // anymal_->getFrameVelocity(footFrameIndex1, footVel1);
    // anymal_->getFrameVelocity(footFrameIndex2, footVel2);
    // anymal_->getFrameVelocity(footFrameIndex3, footVel3);
    // anymal_->getFrameVelocity(footFrameIndex4, footVel4);
    // Eigen::Vector3d bodyfootVel1 = rot_mat.e().transpose() * footVel1.e();
    // Eigen::Vector3d bodyfootVel2 = rot_mat.e().transpose() * footVel2.e();
    // Eigen::Vector3d bodyfootVel3 = rot_mat.e().transpose() * footVel3.e();
    // Eigen::Vector3d bodyfootVel4 = rot_mat.e().transpose() * footVel4.e();
    // double r2 = (bodyfootVel1[0]+bodyfootVel2[0]+bodyfootVel3[0]+bodyfootVel4[0])/4;
    // // double r2 = (std::min(bodyfootVel1[0],0.35)+std::min(bodyfootVel2[0],0.35)+
    // //                 std::min(bodyfootVel3[0],0.35)+std::min(bodyfootVel4[0],0.35))/4;
    // double r_feet = std::min(k*r2,r2);
    // rewards_.record("r_feet",r_feet);
    // // 3. Up reward
    // Eigen::Vector3d ea = rot_mat.e().eulerAngles(0,1,2);
    // double v_up = ea(Eigen::seq(0,1)).norm();
    // double w_up = atanh(sqrt(0.95))/0.4;
    // double c_prec_up = tanh((v_up*w_up) * (v_up *w_up));
    // double r_up = 1-c_prec_up;
    // rewards_.record("r_up",r_up);
    // // 4. energy penalty
    // rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());

    // rewards_.record("forwardVel", std::min(0.5, bodyLinearVel_[0]));
    // rewards_.record("angVel", abs(bodyAngularVel_[2]));

    // std::cout<< anymal_->getGeneralizedForce().squaredNorm()*(6e-5)<< " vs "<<std::min(0.5, bodyLinearVel_[0])<< " vs "<<abs(bodyAngularVel_[2])/2<<std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
    obDouble_ << gc_[2], /// body height
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12); /// joint velocity
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
  bool solver(std::vector<double>& solution, double d, double h1, double h2, double m1, double m2,
            double target_a, double delta){
    double error = 0.01;
    Eigen::Matrix<double,4,4> J;
    Eigen::Vector4d a;
    Eigen::Vector4d F;
    bool solved = false;
    bool valid_slope = true;
    int count = 0;
    while((!solved || !valid_slope) && count<100){
        count++;
        //initialize varaibles
        a = Eigen::Vector4d::Random();
        for(int i= 0;i<10000;i++){     
            //caculate Jacobian and Function
            J(0,0) = sin(a(2));
            J(0,1) = 0;
            J(0,2) = a(0)*cos(a(2));
            J(0,3) = 1;

            J(1,0) = sin(a(1)*d+a(2));
            J(1,1) = a(0)*d*cos(a(1)*d+a(2));
            J(1,2) = a(0)*cos(a(1)*d+a(2));
            J(1,3) = 1;

            J(2,0) = a(1)*cos(a(2));
            J(2,1) = a(0)*cos(a(2));
            J(2,2) = -1*a(0)*a(1)*sin(a(2));
            J(2,3) = 0;

            J(3,0) = a(1)*cos(a(1)*d+a(2));
            J(3,1) = a(0)*cos(a(1)*d+a(2))-a(0)*a(1)*d*sin(a(1)*d+a(2));
            J(3,2) = -1*a(0)*a(1)*sin(a(1)*d+a(2));
            J(3,3) = 0;

            F(0) = a(0)*sin(a(2))+a(3)-h1;
            F(1) = a(0)*sin(a(1)*d+a(2))+a(3)-h2;
            F(2) = a(0)*a(1)*cos(a(2))-m1;
            F(3) = a(0)*a(1)*cos(a(1)*d+a(2))-m2;
            
            //Check if J is invertible
            if(J.determinant()<1e-14 && J.determinant()>-1*(1e-14)) {
                solved=false;
                break;
            }
            // if J is invertible, do update
            else{
                a << a - J.inverse()*F;
                // if solution found, get out of the loop.
                F(0) = a(0)*sin(a(2))+a(3)-h1;
                F(1) = a(0)*sin(a(1)*d+a(2))+a(3)-h2;
                F(2) = a(0)*a(1)*cos(a(2))-m1;
                F(3) = a(0)*a(1)*cos(a(1)*d+a(2))-m2;
                if(F(0) < error && F(0) > -1*error && F(1) < error && F(1) >-1*error &&
                    F(2) < error && F(2) > -1*error && F(3) < error && F(3) > -1*error)
                    {
                        solved = true;
                        break;
                    }
            }
        }
        // after solving, check if the slope is in the valid range
        if(solved){
            for(double curr_d=delta;curr_d<=d;curr_d+=delta){
                double delta_h = a(0)*(sin(a(1)*curr_d+a(2)) - sin(a(1)*(curr_d-delta)+a(2)));
                double a = atan(delta_h/delta);
                // check if slope is in the range
                if(a>target_a*M_PI/180 || a<-1*target_a*M_PI/180){
                    valid_slope = false;
                    break;
                }
            }
        }
    }

    solution.push_back(a(0));
    solution.push_back(a(1));
    solution.push_back(a(2));
    solution.push_back(a(3));
    return (solved && valid_slope);
}

  int gen_combined_terrain(std::vector<double>& chosen_height, std::vector<double>& chosen_depth, 
                            double target_h,double target_a, double delta){
    //define constants
    //4-levels
    double height_diff[4][2] = {{-1*target_h/4,target_h/4},{-2*target_h/4,2*target_h/4},
                                {-3*target_h/4,3*target_h/4},{-1*target_h,target_h}};
    double angles[4][2] = {{-M_PI/180*target_a/4,M_PI/180*target_a/4},{-2*M_PI/180*target_a/4,2*M_PI/180*target_a/4},
                            {-3*M_PI/180*target_a/4,3*M_PI/180*target_a/4},{-M_PI/180*target_a,M_PI/180*target_a}};
    std::vector<double> depth;
    for(double curr_depth=1.5;curr_depth<=2;curr_depth+=delta) depth.push_back(curr_depth);
    //seed and define distributions
    std::random_device rand_device;
    // std::mt19937 engine(rand_device());
    std::mt19937 engine(1);
    std::uniform_real_distribution<> h_dis[]= {std::uniform_real_distribution<>(height_diff[0][0], height_diff[0][1]),
                                                std::uniform_real_distribution<>(height_diff[1][0], height_diff[1][1]),
                                                std::uniform_real_distribution<>(height_diff[2][0], height_diff[2][1]),
                                                std::uniform_real_distribution<>(height_diff[3][0], height_diff[3][1])};
    std::uniform_real_distribution<> a_dis[]= {std::uniform_real_distribution<>(angles[0][0], angles[0][1]),
                                                std::uniform_real_distribution<>(angles[1][0], angles[1][1]),
                                                std::uniform_real_distribution<>(angles[2][0], angles[2][1]),
                                                std::uniform_real_distribution<>(angles[3][0], angles[3][1])};
    std::uniform_int_distribution<int> which_one(0,2);// 0: stair  1: slope  2: rolling
    std::uniform_int_distribution<int> which_one_prev_rolling(1,2);// 1: slope  2: rolling
    //define variables
    bool invalid_h = false;
    bool is_target = false;
    int level=0;
    bool prev_rolling = false;
    double a_his=0;
    int count = 0;
    //make terrain
    //make the starting terrain(2m in x direction)
    for(int i=0;i<2/delta;i++){
        chosen_height.push_back(0);
    }
    while(true){
        //update level
        double total_depth = (chosen_height.size()-1)*delta;
        if (total_depth>=(50+2)/4 && total_depth<(50+2)*2/4) level=1;
        else if(total_depth>=(50+2)*2/4 && total_depth<(50+2)*3/4) level=2;
        else if(total_depth>=(50+2)*3/4) level=3;
        //Valid case
        if(total_depth>=(49+2) && total_depth<=(51+2) && (!invalid_h) && is_target){
            break;
        }   
        //Invalid case: initialized all variables
        if(total_depth>(51+2) || invalid_h){
            chosen_depth.clear();
            chosen_height.clear();
            std::vector<double>().swap(chosen_depth);
            std::vector<double>().swap(chosen_height);
            //initialize start plane
            for(int i=0;i<2/delta;i++){
                chosen_height.push_back(0);
            }
            invalid_h=false;
            is_target=false;
            level=0;
            prev_rolling = false;
            a_his = 0;
            count = 0;
        }           
        //make terrain  
        else{
            double curr_height = chosen_height.back();
            if(!prev_rolling){
                int w = which_one(engine);
                //stair case 
                if(w==0){
                    //sample height
                    double h = h_dis[level](engine);
                    if(level==0) while(h==0) h = h_dis[level](engine);
                    else{
                        while(h==0 || (height_diff[level-1][0]<=h && h<=height_diff[level-1][1])) h = h_dis[level](engine);
                    }
                    //sample depth
                    std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                    double d = chosen_depth.back();
                    curr_height+= h;
                    //check if height is negative
                    if(curr_height<0) {
                        invalid_h=true;
                        continue;
                    }
                    //calculate stair height distanced by delta and add height to chosen height
                    for(double curr_depth=0;curr_depth<=d;curr_depth+=delta){
                        chosen_height.push_back(curr_height);
                    }   
                    //record angle history
                    a_his = 0;               
                }
                //slope case
                else if(w==1){
                    //smaple angle and depth
                    std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                    double d = chosen_depth.back();
                    double angle = a_dis[level](engine);
                    if(level==0) while(angle==0) angle = a_dis[level](engine);
                    else{
                        while(angle==0 || (angles[level-1][0]<=angle && angle<=angles[level-1][1])) angle = a_dis[level](engine);
                    }
                    // record angle history
                    a_his = angle;    
                    //check if target angle is included
                    if(angle >= angles[level][1]-0.001 || angle <=angles[level][0]+0.001) is_target=true;
                    //calculate slope height distanced by delta and add height to chosen height
                    for(double curr_depth=0;curr_depth<=d;curr_depth+=delta){
                        curr_height+=tan(angle)*delta;
                        //check if negative height
                        if(curr_height<0) {invalid_h=true;break;}
                        chosen_height.push_back(curr_height);
                    }                
                    // check if negative height
                    if(invalid_h) continue;   
                }  
                //rolling case
                else{
                    prev_rolling = true;
                    //sample depth
                    std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                    double d = chosen_depth.back();
                    //sample height
                    double h = h_dis[level](engine);
                    if(level==0) while(h==0) h = h_dis[level](engine);
                    else{
                        while(h==0 || (height_diff[level-1][0]<=h && h<=height_diff[level-1][1])) h = h_dis[level](engine);
                    }
                    curr_height+=h;
                    if(curr_height<0) {
                        invalid_h=true;
                        continue;
                    }
                    //compute next angle
                    double next_angle;
                    w = which_one(engine);
                    if(w==0) next_angle = 0;
                    else {
                        next_angle = a_dis[level](engine);
                        if(level==0) while(next_angle==0) next_angle = a_dis[level](engine);
                        else{
                            while(next_angle==0 || (angles[level-1][0]<=next_angle && next_angle<=angles[level-1][1])) next_angle = a_dis[level](engine);
                        }
                    }
                    //compute sinusoidal graph
                    std::vector<double> coeff;
                    bool solved = solver(coeff,d,chosen_height.back(),curr_height,tan(M_PI/180*a_his),
                                        tan(M_PI/180*next_angle),target_a,delta);
                    if(!solved){
                        prev_rolling = false;
                        chosen_depth.pop_back();
                        curr_height -= h;
                        continue;
                    }
                    a_his = next_angle;
                    //calculate slope height distanced by delta and add height to chosen height
                    for(double curr_depth=delta;curr_depth<=d;curr_depth+=delta){
                        chosen_height.push_back(coeff[0]*sin(coeff[1]*curr_depth+coeff[2])+coeff[3]);
                    }         
                    count++;
                }
            }
            //if previous is rolling terrain, the starting angle of the next terrain is already determined
            else{
                prev_rolling = false;
                // stair case
                if(a_his==0){
                    //sample height and depth
                    double h = h_dis[level](engine);
                    if(level==0) while(h==0) h = h_dis[level](engine);
                    else{
                        while(h==0 || (height_diff[level-1][0]<=h && h<=height_diff[level-1][1])) h = h_dis[level](engine);
                    }
                    std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                    double d = chosen_depth.back();
                    curr_height+= h;
                    //check if height is negative
                    if(curr_height<0) {
                        invalid_h=true;
                        continue;
                    }
                    //calculate stair height distanced by delta and add height to chosen height
                    for(double curr_depth=0;curr_depth<=d;curr_depth+=delta){
                        chosen_height.push_back(curr_height);
                    }   
                    //record angle history
                    a_his = 0;
                }
                // slope or rolling case
                else{
                    int w = which_one_prev_rolling(engine);
                    //slope case
                    if(w==1) {
                        //smaple angle and depth
                        std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                        double d = chosen_depth.back();
                        double angle = a_his; 
                        //check if target angle is included
                        if(angle >= angles[level][1]-0.001 || angle <=angles[level][0]+0.001) is_target=true;
                        //calculate slope height distanced by delta and add height to chosen height
                        for(double curr_depth=0;curr_depth<=d;curr_depth+=delta){
                            curr_height+=tan(angle)*delta;
                            //check if negative height
                            if(curr_height<0) {invalid_h=true;break;}
                            chosen_height.push_back(curr_height);
                        }                
                        // check if negative height
                        if(invalid_h) continue; 
                    }
                    //rolling case
                    else {
                        prev_rolling = true;
                        //sample depth
                        std::sample(depth.begin(),depth.end(),std::back_inserter(chosen_depth),1,engine);
                        double d = chosen_depth.back();
                        //sample height
                        double h = h_dis[level](engine);
                        if(level==0) while(h==0) h = h_dis[level](engine);
                        else{
                            while(h==0 || (height_diff[level-1][0]<=h && h<=height_diff[level-1][1])) h = h_dis[level](engine);
                        }
                        curr_height+=h;
                        if(curr_height<0) {
                            invalid_h=true;
                            continue;
                        }
                        //compute next angle
                        double next_angle;
                        w = which_one(engine);
                        if(w==0) next_angle = 0;
                        else {
                            next_angle = a_dis[level](engine);
                            if(level==0) while(next_angle==0) next_angle = a_dis[level](engine);
                            else{
                                while(next_angle==0 || (angles[level-1][0]<=next_angle && next_angle<=angles[level-1][1])) next_angle = a_dis[level](engine);
                            }
                        }                        
                        //compute sinusoidal graph
                        std::vector<double> coeff;
                        bool solved = solver(coeff,d,chosen_height.back(),curr_height,tan(M_PI/180*a_his),
                                            tan(M_PI/180*next_angle),target_a,delta);
                        if(!solved){
                            prev_rolling = false;
                            chosen_depth.pop_back();
                            curr_height -= h;
                            continue;
                        }   
                        a_his = next_angle;
                        //calculate slope height distanced by delta and add height to chosen height
                        for(double curr_depth=delta;curr_depth<=d;curr_depth+=delta){
                            chosen_height.push_back(coeff[0]*sin(coeff[1]*curr_depth+coeff[2])+coeff[3]);
                        } 
                        count++;
                    }
                }
            }        
        }
    } 
    return count;
}

  

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

