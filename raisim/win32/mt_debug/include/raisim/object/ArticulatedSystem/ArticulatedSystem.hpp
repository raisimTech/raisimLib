//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_ARTICULATEDSYSTEM_HPP
#define RAISIM_ARTICULATEDSYSTEM_HPP

#include <vector>
#include <initializer_list>
#include <string>
#include <array>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <fstream>
#include "raisim/math.hpp"
#include "algorithm"

#include "JointAndBodies.hpp"
#include "raisim/object/singleBodies/SingleBodyObject.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"

namespace raisim {

namespace mjcf {
class LoadFromMJCF;
}

namespace urdf {
class LoadFromURDF2;
}

namespace ControlMode {
enum Type :
    int {
  FORCE_AND_TORQUE = 0,
  PD_PLUS_FEEDFORWARD_TORQUE
};
}

class ArticulatedSystemOption {
 public:
  bool doNotCollideWithParent = true;
};

class ArticulatedSystem : public Object {

  /* list of vocabs
     1. body: body here refers to only rotating bodies. Fixed bodies are optimized out. 
              Position of a body refers to the position of the joint connecting the body and its parent. 
     2. Coordinate frame: coordinate frames are defined on every joint (even at the fixed joints). If you want
                          to define a custom frame, define a fixed zero-mass object and a joint in the URDF */

 public:

  enum Frame :
      unsigned {
    WORLD_FRAME = 0,
    PARENT_FRAME,
    BODY_FRAME
  };

  enum class IntegrationScheme : int {
    TRAPEZOID = 0, /// after this, only second order or higher
    SEMI_IMPLICIT,
    EULER,
    RUNGE_KUTTA_4
  };

  bool isSecondOrderOrHigher(IntegrationScheme scheme) { return int(scheme) > int(IntegrationScheme::EULER); }

  struct SpringElement {
    SpringElement() { q_ref.setZero(); }

    /// the spring connects this body and its parent
    size_t childBodyId = 0;
    /// spring stiffness
    double stiffness = 0;
    /// mounting angles for torsional and linear springs
    Vec<4> q_ref;
  };

  class LinkRef {
   public:
    LinkRef(size_t localId, ArticulatedSystem* system) :
      system_(system), localId_(localId) {

      for(auto& col: system_->getCollisionBodies()) {
        if(col.localIdx==localId_) {
          colDef_.insert({col.name, &col});
        }
      }

      for(auto& vis: system_->visObj) {
        if(vis.localIdx == localId_) {
          visDef_.insert({vis.name, &vis});
        }
      }
    }

    void getPosition(Vec<3>& position) {
      system_->getBodyPosition(localId_, position);
    }

    void getOrientation(Mat<3,3>& orientation) {
      system_->getBodyOrientation(localId_, orientation);
    }

    void getPose(Vec<3>& position, Mat<3,3>& orientation) {
      system_->getBodyPose(localId_, orientation, position);
    }

    CollisionDefinition* getCollisionDefinition(const std::string& name) {
      return colDef_[name];
    }

    VisObject* getVisualObject(const std::string& name) {
      return visDef_[name];
    }

    void setWeight(double weight) {
      system_->getMass()[localId_] = weight;
      system_->updateMassInfo();
    }

    double getWeight() {
      return system_->getMass(localId_);
    }

    void setInertia(const Mat<3,3>& inertia) {
      system_->getInertia()[localId_] = inertia;
    }

    const Mat<3,3>& getInertia() {
      return system_->getInertia()[localId_];
    }

    /* set center of mass position in parent frame */
    void setComPositionInParentFrame(const Vec<3>& com) {
      system_->getLinkCOM()[localId_] = com;
    }

    /* get center of mass position in parent frame */
    const Vec<3>& getComPositionInParentFrame() {
      return system_->getLinkCOM()[localId_];
    }

    const std::unordered_map<std::string, CollisionDefinition*>& getCollisionSet() {
      return colDef_;
    }

    const std::unordered_map<std::string, VisObject*>& getVisualSet() {
      return visDef_;
    }

   private:
    raisim::ArticulatedSystem* system_;
    size_t localId_;
    std::unordered_map<std::string, CollisionDefinition*> colDef_;
    std::unordered_map<std::string, VisObject*> visDef_;
  };

  class JointRef {
   public:
    JointRef(size_t frameId, ArticulatedSystem* system) :
        system_(system), frameId_(frameId) {
      auto& toGvIndex = system_->getMappingFromBodyIndexToGeneralizedVelocityIndex();
      auto& toGcIndex = system_->getMappingFromBodyIndexToGeneralizedCoordinateIndex();

      isMovable_ = system_->getFrames()[frameId].isChild;

      if(isMovable_) {
        gvIndx_ = toGvIndex[system_->getFrames()[frameId].parentId];
        gcIndx_ = toGcIndex[system_->getFrames()[frameId].parentId];
      } else {
        gvIndx_ = -1;
        gcIndx_ = -1;
      }

      jointType_ = system_->getFrames()[frameId].jointType;
    }

    void getPosition(Vec<3>& position) {
      system_->getFramePosition(frameId_, position);
    }

    Mat<3,3> getOrientation() {
      Mat<3,3> orientation;
      system_->getFrameOrientation(frameId_, orientation);
      return orientation;
    }

    void getPose(Vec<3>& position, Mat<3,3>& orientation) {
      system_->getFramePosition(frameId_, position);
      system_->getFrameOrientation(frameId_, orientation);
    }

    /* Joint coordinate can be multiple dimensional vector (for a spherical joint)
     * or 1D vector */
    void getJointCoordinate(VecDyn& coordinate) {
      if(jointType_ == raisim::Joint::Type::SPHERICAL) {
        coordinate.resize(4);
        coordinate[0] = system_->getGeneralizedCoordinate()[gcIndx_];
        coordinate[1] = system_->getGeneralizedCoordinate()[gcIndx_+1];
        coordinate[2] = system_->getGeneralizedCoordinate()[gcIndx_+2];
        coordinate[3] = system_->getGeneralizedCoordinate()[gcIndx_+3];
      }

      coordinate[0] = system_->getGeneralizedCoordinate()[gcIndx_];
    }


    double getJointAngle() {
      return system_->getGeneralizedCoordinate()[gcIndx_];
    }

    raisim::Vec<3>& getPositionInParentFrame() {
      if(isMovable_)
        return system_->getJointPos_P()[gvIndx_];
      else
        RSFATAL("This is a fixed joint. You cannot change the position of a fixed joint.")
    }

    raisim::Vec<3>& getJointAxis() {
      if(isMovable_)
        return system_->getJointAxis_P()[gvIndx_];
      else
        RSFATAL("This is a fixed joint. You cannot change the position of a fixed joint.")
    }

    Joint::Type getType() {
      return system_->getJointType(gvIndx_);
    }

    size_t getIdxInGeneralizedCoordinate() {
      return gvIndx_;
    }

    Vec<3> getLinearVelocity() {
      Vec<3> linVel;
      system_->getFrameVelocity(frameId_, linVel);
      return linVel;
    }

   private:
    raisim::ArticulatedSystem* system_;
    size_t frameId_, gvIndx_, gcIndx_;
    bool isMovable_;
    Joint::Type jointType_;

  };

 private:
  friend class raisim::World;
  friend class raisim::urdf::LoadFromURDF2;
  friend class raisim::mjcf::LoadFromMJCF;

 public:

  typedef Eigen::Map<Eigen::Matrix<double, -1, 1> > EigenVec;
  typedef Eigen::Map<Eigen::Matrix<double, -1, -1> > EigenMat;

  ArticulatedSystem() = default;

  ArticulatedSystem(const Child& child,
                    const std::string& resDir,
                    ArticulatedSystemOption options);

  ArticulatedSystem(const std::string &filePathOrURDFScript,
                    const std::string &resDir="",
                    const std::vector<std::string> &jointOrder = std::vector<std::string>(),
                    ArticulatedSystemOption options = ArticulatedSystemOption());

  ~ArticulatedSystem();

  /**
   * @return generalized coordinate of the system */
  const raisim::VecDyn &getGeneralizedCoordinate() { return gc_; }

  /**
   * @return generalized velocity of the system */
  const raisim::VecDyn &getGeneralizedVelocity() { return gv_; }

  /**
   * @param[out] quaternion orientation of base*/
  void getBaseOrientation(raisim::Vec<4>& quaternion);

  /**
   * @param[out] rotataionMatrix orientation of base*/
  void getBaseOrientation(raisim::Mat<3,3>& rotataionMatrix) { rotataionMatrix = rot_WB[0]; }

  /**
   * @return orientation of base*/
  const raisim::Mat<3,3>& getBaseOrientation() { return rot_WB[0]; }

  /**
   * @param[out] position position of base*/
  void getBasePosition(raisim::Vec<3>& position) {
    RSFATAL_IF(jointType[0]!=Joint::Type::FLOATING, "This method is only for floating base")
    position[0] = gc_[0]; position[1] = gc_[1]; position[2] = gc_[2];
  }

  /**
   * @return position of base*/
  raisim::Vec<3> getBasePosition() {
    RSFATAL_IF(jointType[0]!=Joint::Type::FLOATING, "This method is only for floating base")
    raisim::Vec<3> position;
    position[0] = gc_[0]; position[1] = gc_[1]; position[2] = gc_[2];
    return position;
  }

  /**
   * unnecessary to call this function if you are simulating your system. integrate1 calls this function
   * Call this function if you want to get kinematic properties but you don't want to integrate.  */
  void updateKinematics();

  /**
   * set gc of each joint in order
   * @param[in] jointState generalized coordinate */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) { gc_ = jointState; updateKinematics(); cleanContacts(); }
  /**
   * set gc of each joint in order
   * @param[in] jointState generalized coordinate */
  void setGeneralizedCoordinate(const raisim::VecDyn &jointState) { gc_ = jointState; updateKinematics(); cleanContacts(); }

  /**
   * set the generalized velocity
   * @param[in] jointVel the generalized velocity*/
  void setGeneralizedVelocity(const Eigen::VectorXd &jointVel) { gv_ = jointVel; }

  /**
   * set the generalized velocity
   * @param[in] jointVel the generalized velocity*/
  void setGeneralizedVelocity(const raisim::VecDyn &jointVel) { gv_ = jointVel; }

  /**
   * set the generalized coordinsate of each joint in order.
   * @param[in] jointState the generalized coordinate */
  void setGeneralizedCoordinate(std::initializer_list<double> jointState);

  /**
   * set the generalized velocity of each joint in order
   * @param[in] jointState the generalized velocity */
  void setGeneralizedVelocity(std::initializer_list<double> jointVel);

  /**
   * This is feedforward generalized force. In the PD control mode, this differs from the actual generalizedForce
   * the dimension should be the same as dof.
   * @param[in] tau the generalized force. If the built-in PD controller is active, this force is added to the generalized force from the PD controller*/
  void setGeneralizedForce(std::initializer_list<double> tau);
  /**
   * This is feedforward generalized force. In the PD control mode, this differs from the actual generalizedForce
   * the dimension should be the same as dof.
   * @param[in] tau the generalized force. If the built-in PD controller is active, this force is added to the generalized force from the PD controller*/
  void setGeneralizedForce(const raisim::VecDyn &tau) { tauFF_ = tau; }
  /**
   * This is feedforward generalized force. In the PD control mode, this differs from the actual generalizedForce
   * the dimension should be the same as dof.
   * @param[in] tau the generalized force. If the built-in PD controller is active, this force is added to the generalized force from the PD controller*/
  void setGeneralizedForce(const Eigen::VectorXd &tau) { tauFF_ = tau; }

  /**
   * get both the generalized coordinate and the generalized velocity
   * @param[out] genco the generalized coordinate
   * @param[out] genvel the generalized velocity */
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) { genco = gc_.e(); genvel = gv_.e();}

  /**
   * get both the generalized coordinate and the generalized velocity
   * @param[out] genco the generalized coordinate
   * @param[out] genvel the generalized velocity */
  void getState(VecDyn &genco, VecDyn &genvel) { genco = gc_; genvel = gv_;}

  /**
   * set both the generalized coordinate and the generalized velocity. This updates the kinematics and removes previously computed contact points
   * @param[in] genco the generalized coordinate
   * @param[in] genvel the generalized velocity */
  void setState(const Eigen::VectorXd &genco, const Eigen::VectorXd &genvel) {
    gc_ = genco;
    gv_ = genvel;
    updateKinematics();
    cleanContacts();
  }

  /* get dynamics properties. Make sure that after integration you call "integrate1()" of the world object before using this method" */

  /**
   * Generalized force is the sum of (the feedforward force) and (the PD controller force after thresholding by the "effort" specified in the urdf)
   * @return the generalized force */
  const VecDyn &getGeneralizedForce() const { return tau_; }

  /**
   * get the feedfoward generalized force (which is set by the user)
   * @return the feedforward generalized force */
  const VecDyn &getFeedForwardGeneralizedForce() const { return tauFF_; }

  /**
   * get the mass matrix
   * @return the mass matrix. Check Object/ArticulatedSystem section in the manual */
  const MatDyn &getMassMatrix() const { return M_; }

  /**
   * get the coriolis and the gravitational term
   * @return the coriolis and the gravitational term. Check Object/ArticulatedSystem section in the manual */
  const VecDyn &getNonlinearities() const { return h_; }

  /**
   * get the inverse mass matrix. Note that this is actually damped inverse. It contains the effect of damping and the spring effects
   * @return the inverse mass matrix. Check Object/ArticulatedSystem section in the manual */
  const MatDyn &getInverseMassMatrix() const { return Minv_; }

  /**
   * get the center of mass of the whole system
   * @return the center of mass */
  const Vec<3> &getCompositeCOM() const { return composite_com_W[0]; }

  /**
   * get the current composite inertia of the whole system. This value assumes all joints are fixed
   * @return the inertia of the coposite system */
  const Mat<3, 3> &getCompositeInertia() const { return compositeInertia_W[0]; }

  /**
   * linear momentum of the whole system
   * @return momentum */
  const Vec<3> &getLinearMomentum();

  /**
   * returns the generalized momentum which is M * u
   * It is already computed in "integrate1()" so you don't have to compute again.
   * @return the generalized momentum */
  const VecDyn &getGeneralizedMomentum() const { return generalizedMomentum_; }

  /**
   * @param[in] gravity gravitational acceleration
   * @return the sum of potential/kinetic energy given the gravitational acceleration*/
  double getEnergy(const Vec<3> &gravity);

  /**
   * @return the kinetic energy. */
  double getKineticEnergy();

  /**
   * @param[in] gravity gravitational acceleration
   * @return the potential energy (relative to zero height) given the gravity vector */
  double getPotentialEnergy(const Vec<3> &gravity);

  /**
   * @param[in] referencePoint the reference point about which the angular momentum is computed
   * @param[in] angularMomentum angular momentum about the reference point*/
  void getAngularMomentum(const Vec<3>& referencePoint, Vec<3> &angularMomentum);

  /**
   * bodies here means moving bodies. Fixed bodies are optimized out*/
  void printOutBodyNamesInOrder() const;

  /**
   * frames are attached to every joint coordinate */
  void printOutFrameNamesInOrder() const;

  /**
   * getMovableJointNames. Note! the order doesn't correspond to dof since there are
   * joints with multiple dof's
   * @return movable joint names in the joint order. */
  const std::vector<std::string>& getMovableJointNames() const {return movableJointNames; };

  /**
   * @param[in] bodyIdx The body which contains the point, can be retrieved by getBodyIdx()
   * @param[in] point_B The position of the point in the body frame
   * @param[out] point_W The position of the point in the world frame
   * @return position in the world frame of a point defined in a joint frame*/
  void getPosition(size_t bodyIdx, const Vec<3> &point_B, Vec<3> &point_W) const final;

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * @param[in] nm name of the frame
   * @return the coordinate frame of the given name */
  CoordinateFrame &getFrameByName(const std::string &nm);

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * @param[in] idx index of the frame
   * @return the coordinate frame of the given index */
  CoordinateFrame &getFrameByIdx(size_t idx);

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * The frame can be retrieved as as->getFrames[index]. This way is more efficient than above methods that use the frame name
   * @param[in] nm name of the frame
   * @return the index of the coordinate frame of the given index. */
  size_t getFrameIdxByName(const std::string &nm) const;

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * The frame can be retrieved as as->getFrames[index]. This way is more efficient than above methods that use the frame name
   * @return a vector of the coordinate frames */
  std::vector<CoordinateFrame> &getFrames();

  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[out] point_W the position of the frame expressed in the world frame */
  void getFramePosition(size_t frameId, Vec<3> &point_W) const;

  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[out] orientation_W the position of the frame relative to the world frame */
  void getFrameOrientation(size_t frameId, Mat<3, 3> &orientation_W) const;

  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[out] vel_W the linear velocity of the frame expressed in the world frame */
  void getFrameVelocity(size_t frameId, Vec<3> &vel_W) const;

  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[out] angVel_W the angular velocity of the frame expressed in the world frame */
  void getFrameAngularVelocity(size_t frameId, Vec<3> &angVel_W) const;

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] point_W the position of the frame expressed in the world frame */
  void getFramePosition(const std::string& frameName, Vec<3> &point_W) const {
    getFramePosition(getFrameIdxByName(frameName), point_W); }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] orientation_W the orientation of the frame relative to the world frame */
  void getFrameOrientation(const std::string& frameName, Mat<3, 3> &orientation_W) const {
    getFrameOrientation(getFrameIdxByName(frameName), orientation_W); }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] vel_W the linear velocity of the frame expressed in the world frame */
  void getFrameVelocity(const std::string& frameName, Vec<3> &vel_W) const {
    getFrameVelocity(getFrameIdxByName(frameName), vel_W); }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] angVel_W the angular velocity of the frame relative to the world frame */
  void getFrameAngularVelocity(const std::string& frameName, Vec<3> &angVel_W) const {
    getFrameAngularVelocity(getFrameIdxByName(frameName), angVel_W); }

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] point_W the position of the frame relative to the world frame */
  void getFramePosition(const CoordinateFrame& frame, Vec<3> &point_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] orientation_W the orientation of the frame relative to the world frame */
  void getFrameOrientation(const CoordinateFrame& frame, Mat<3, 3> &orientation_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] vel_W the linear velocity of the frame expressed to the world frame */
  void getFrameVelocity(const CoordinateFrame& frame, Vec<3> &vel_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] angVel_W the angular velocity of the frame expressed to the world frame */
  void getFrameAngularVelocity(const CoordinateFrame& frame, Vec<3> &angVel_W) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] pos_w the position of the joint (after its own joint transformation) */
  void getPosition(size_t bodyIdx, Vec<3> &pos_w) const final { pos_w = jointPos_W[bodyIdx]; }

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] rot the orientation of the joint (after its own joint transformation) */
  void getOrientation(size_t bodyIdx, Mat<3, 3> &rot) const final { rot = rot_WB[bodyIdx]; }

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] vel_w the linear velocity of the joint (after its own joint transformation) */
  void getVelocity(size_t bodyIdx, Vec<3> &vel_w) const final { vel_w = bodyLinearVel_W[bodyIdx]; }

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] angVel_w the angular velocity of the joint (after its own joint transformation) */
  void getAngularVelocity(size_t bodyIdx, Vec<3> &angVel_w) const { angVel_w = bodyAngVel_W[bodyIdx]; }

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[in] point_W the point expressed in the world frame. If you want to use a point expressed in the body frame, use getDenseFrameJacobian()
   * @param[out] jaco the positional Jacobian. v = J * u. v is the linear velocity expressed in the world frame and u is the generalized velocity */
  void getSparseJacobian(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[in] point_W the point expressed in the world frame. If you want to use a point expressed in the body frame, use getDenseRotationalJacobian()
   * @param[out] jaco the rotational Jacobian. \omega = J * u. \omgea is the angular velocity expressed in the world frame and u is the generalized velocity */
  void getSparseRotationalJacobian(size_t bodyIdx, SparseJacobian &jaco);

  /**
   * @param[in] sparseJaco sparse Jacobian (either positional or rotational)
   * @param[out] denseJaco the corresponding dense Jacobian */
  static void convertSparseJacobianToDense(const SparseJacobian &sparseJaco, Eigen::MatrixXd &denseJaco) {
    for (size_t i = 0; i < sparseJaco.size; i++) {
      denseJaco(0, sparseJaco.idx[i]) = sparseJaco[i * 3];
      denseJaco(1, sparseJaco.idx[i]) = sparseJaco[i * 3 + 1];
      denseJaco(2, sparseJaco.idx[i]) = sparseJaco[i * 3 + 2];
    }
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint
   * @param[in] point_W the point expressed in the world frame. If you want to use a point expressed in the body frame, use getDenseFrameJacobian()
   * @param[out] jaco the dense positional Jacobian */
  void getDenseJacobian(size_t bodyIdx, const Vec<3> &point_W, Eigen::MatrixXd &jaco) {
    DRSFATAL_IF(jaco.rows() != 3 || jaco.cols() != dof, "Jacobian should be in size of 3XDOF")
    SparseJacobian sparseJaco;
    sparseJaco.resize(dof);
    getSparseJacobian(bodyIdx, point_W, sparseJaco);
    for (size_t i = 0; i < sparseJaco.size; i++)
      for (size_t j = 0; j < 3; j++)
        jaco(j, sparseJaco.idx[i]) = sparseJaco[i * 3 + j];
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] jaco the dense rotational Jacobian */
  void getDenseRotationalJacobian(size_t bodyIdx, Eigen::MatrixXd &jaco) {
    DRSFATAL_IF(jaco.rows() != 3 || jaco.cols() != dof, "Jacobian should be in size of 3XDOF")
    SparseJacobian sparseJaco;
    sparseJaco.resize(dof);
    getSparseRotationalJacobian(bodyIdx, sparseJaco);
    for (size_t i = 0; i < sparseJaco.size; i++)
      for (size_t j = 0; j < 3; j++)
        jaco(j, sparseJaco.idx[i]) = sparseJaco[i * 3 + j];
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameIdx the frame index. it can be retrieved by getFrameIdxByName()
   * @param[out] jaco the dense positional Jacobian */
  void getDenseFrameJacobian(size_t frameIdx, Eigen::MatrixXd &jaco) {
    auto& frame = getFrameByIdx(frameIdx);
    Vec<3> position_W;
    getFramePosition(frameIdx, position_W);
    getDenseJacobian(frame.parentId, position_W, jaco);
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameName the frame name. (defined in the URDF)
   * @param[out] jaco the dense positional Jacobian */
  void getDenseFrameJacobian(const std::string& frameName, Eigen::MatrixXd &jaco) {
    getDenseFrameJacobian(getFrameIdxByName(frameName), jaco); }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameIdx the frame index. it can be retrieved by getFrameIdxByName()
   * @param[out] jaco the dense rotational Jacobian */
  void getDenseFrameRotationalJacobian(size_t frameIdx, Eigen::MatrixXd &jaco) {
    auto& frame = getFrameByIdx(frameIdx);
    getDenseRotationalJacobian(frame.parentId, jaco);
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameName the frame name. (defined in the URDF)
   * @param[out] jaco the dense rotational Jacobian */
  void getDenseFrameRotationalJacobian(const std::string& frameName, Eigen::MatrixXd &jaco) {
    getDenseFrameRotationalJacobian(getFrameIdxByName(frameName), jaco); }

  /**
   * @param[in] jaco the Jacobian associated with the point of interest
   * @param[out] pointVel the velocity of the point expressed in the world frame */
  void getVelocity(const SparseJacobian &jaco, Vec<3> &pointVel) const;

  /**
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] posInBodyFrame the position of the point of interest expressed in the body frame
   * @param[out] pointVel the velocity of the point expressed in the world frame */
  void getVelocity(size_t bodyIdx, const Vec<3> &posInBodyFrame, Vec<3> &pointVel) const;

  /**
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] angVel the angular velocity of the point expressed in the world frame */
  void getAngularVelocity(size_t bodyIdx, Vec<3> &angVel);

  /**
   * returns the index of the body
   * @param[in] nm name of the body. The body name is the name of the movable link of the body
   * @return the index of the body */
  size_t getBodyIdx(const std::string &nm);

  /**
   * @return the degrees of freedom */
  size_t getDOF() const;

  /**
   * @return the dimension of generalized velocity (do the same thing with getDOF) */
  size_t getGeneralizedVelocityDim() const;

  /**
   * @return the dimension of generalized coordinate */
  size_t getGeneralizedCoordinateDim() const;

  /**
   * The body pose is the pose of its parent joint (after its joint transformation)
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] orientation the orientation of the body
   * @param[out] position the position of the body */
  void getBodyPose(size_t bodyIdx, Mat<3, 3> &orientation, Vec<3> &position);

  /**
   * The body position is the position of its parent joint (after its joint transformation)
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] position the position of the body */
  void getBodyPosition(size_t bodyIdx, Vec<3> &position);

  /**
   * The body orientation is the orientation of its parent joint (after its joint transformation)
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] orientation the orientation of the body */
  void getBodyOrientation(size_t bodyIdx, Mat<3, 3> &orientation);

  /**
   * The following 5 methods can be used to directly modify dynamic/kinematic properties of the robot.
   * They are made for dynamic randomization. Use them with caution since they will change the
   * the model permenantly. After you change the dynamic properties, call "void updateMassInfo()" to update
   * some precomputed dynamic properties */
  /**
   * @return a non-const reference to joint position relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Vec<3>> &getJointPos_P() { return jointPos_P; }

  /**
   * @return a non-const reference to joint orientation relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Mat<3,3>> &getJointOrientation_P() { return rot_JB; }

  /**
   * @return a non-const reference to joint axis relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Vec<3>> &getJointAxis_P() { return jointAxis_P; }

  /**
   * You MUST call updateMassInfo() after you change the mass
   * @return a non-const reference to mass of each joint.*/
  std::vector<double> &getMass() { return mass; }

  /**
   * @return a non-const reference to inertia of each body.*/
  std::vector<raisim::Mat<3, 3>> &getInertia() { return inertia_comB; }

  /**
   * @return a non-const reference to the position of the center of the mass in the body frame.*/
  std::vector<raisim::Vec<3>> &getLinkCOM() { return comPos_B; }

  /**
   * @return a non-const reference to the collision bodies. Position and orientation can be set dynamically */
  raisim::CollisionSet &getCollisionBodies() { return collisionBodies; }

  /**
   * @param name collision body name which is "LINK_NAME" + "/" + "COLLISION_NUMBER". For example, the first collision body of the link "base" is named as "base/0"
   * @return a non-const reference to the collision bodies. Position and orientation can be set dynamically */
  raisim::CollisionDefinition &getCollisionBody(const std::string& name) {
    return *std::find_if(collisionBodies.begin(), collisionBodies.end(),
                      [name](const raisim::CollisionDefinition& ref){ return ref.name == name; }); }

  /**
   * This method updates the precomputed composite mass. Call this method after you change link mass */
  void updateMassInfo();

  /**
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @return mass of the body */
  double getMass(size_t bodyIdx) const final { return mass[bodyIdx]; }

  /**
   * set body mass. It is indexed for each body, no each link */
  void setMass(size_t bodyIdx, double value) { mass[bodyIdx] = value; }

  /**
   * @return the total mass of the system*/
  double getTotalMass() { return compositeMass[0]; }

  /**
   * set external forces or torques expressed in the world frame acting on the COM of the body
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] force force applied to the body (at the center of mass) */
  void setExternalForce(size_t bodyIdx, const Vec<3> &force) final;

  /** set external force acting on the point specified
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] frameOfForce the frame in which the force is expressed. Options: Frame::WORLD_FRAME, Frame::PARENT_FRAME or Frame::BODY_FRAME
   * @param[in] force the applied force
   * @param[in] frameOfPos the frame in which the position vector is expressed. Options: Frame::WORLD_FRAME, Frame::PARENT_FRAME or Frame::BODY_FRAME
   * @param[in] pos position at which the force is applied*/
  void setExternalForce(size_t bodyIdx, Frame frameOfForce, const Vec<3> &force, Frame frameOfPos, const Vec<3> &pos);

  /** set external force (expressed in the world frame) acting on the point (expressed in the body frame) specified
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] force the applied force
   * @param[in] pos position at which the force is applied*/
  void setExternalForce(size_t bodyIdx, const Vec<3>& pos, const Vec<3>& force) final {
    setExternalForce(bodyIdx, Frame::WORLD_FRAME, force, Frame::BODY_FRAME, pos);
  }

  /** set external force (expressed in the world frame) acting on the point specified by the frame
   * @param[in] frame the name of the frame where you want to applied the force. The force is applied to the origin of the frame, on the body where the frame is attached to.
   * @param[in] force the applied force in the world frame*/
  void setExternalForce(const std::string& frame_name, const Vec<3>& force) {
    auto& frame = getFrameByName(frame_name);
    setExternalForce(frame.parentId, Frame::WORLD_FRAME, force, Frame::BODY_FRAME, frame.position);
  }

  /**
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] torque_in_world_frame the applied torque expressed in the world frame */
  void setExternalTorque(size_t bodyIdx, const Vec<3> &torque_in_world_frame) final;

  /**
   * returns the contact point velocity. The contactId is the order in the vector from Object::getContacts()
   * @param[in] contactId index of the contact vector which can be obtained by getContacts()
   * @param[out] vel the contact point velocity */
  void getContactPointVel(size_t contactId, Vec<3> &vel) final;

  /**
   * @param[in] mode control mode. Can be either ControlMode::FORCE_AND_TORQUE or ControlMode::PD_PLUS_FEEDFORWARD_TORQUE */
  void setControlMode(ControlMode::Type mode) { controlMode_ = mode; }

  /**
   * @return control mode. Can be either ControlMode::FORCE_AND_TORQUE or ControlMode::PD_PLUS_FEEDFORWARD_TORQUE */
  ControlMode::Type getControlMode() { return controlMode_; }

  /**
   * set PD targets.
   * @param[in] posTarget position target
   * @param[in] velTarget velocity target */
  void setPdTarget(const Eigen::VectorXd &posTarget, const Eigen::VectorXd &velTarget) {
    RSFATAL_IF(posTarget.rows() != gcDim,"position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(velTarget.rows() != dof, "the velocity target should have the same dimension as the degrees of freedom")
    qref_ = posTarget;
    uref_ = velTarget;
  }

  /**
   * set PD targets.
   * @param[in] posTarget position target (dimension == getGeneralizedCoordinateDim())
   * @param[in] velTarget velocity target (dimension == getDOF()) */
  void setPdTarget(const raisim::VecDyn &posTarget, const raisim::VecDyn &velTarget) {
    RSFATAL_IF(posTarget.rows() != gcDim,"position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(velTarget.rows() != dof, "the velocity target should have the same dimension as the degrees of freedom")
    qref_ = posTarget;
    uref_ = velTarget;
  }

  /**
   * set P targets.
   * @param[in] posTarget position target (dimension == getGeneralizedCoordinateDim())*/
  template<class T>
  void setPTarget(const T &posTarget) {
    setControlMode(ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    RSFATAL_IF(posTarget.rows() != gcDim,"position target should have the same dimension as the generalized coordinate")
    qref_ = posTarget;
  }

  /**
   * set D targets.
   * @param[in] velTarget velocity target (dimension == getDOF()) */
  template<class T>
  void setDTarget(const T &velTarget) {
    RSFATAL_IF(velTarget.rows() != dof, "the velocity target should have the same dimension as the degrees of freedom")
    uref_ = velTarget;
  }

  /**
   * set PD gains.
   * @param[in] pgain position gain (dimension == getDOF())
   * @param[in] dgain velocity gain (dimension == getDOF())*/
  void setPdGains(const Eigen::VectorXd &pgain, const Eigen::VectorXd &dgain) {
    setControlMode(ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    setPGains(pgain);
    setDGains(dgain);
  }

  /**
   * set PD gains.
   * @param[in] pgain position gain (dimension == getDOF())
   * @param[in] dgain velocity gain (dimension == getDOF())*/
  void setPdGains(const raisim::VecDyn &pgain, const raisim::VecDyn &dgain) {
    setControlMode(ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    setPGains(pgain);
    setDGains(dgain);
  }

  /**
   * set P gain.
   * @param[in] pgain position gain (dimension == getDOF())*/
  template<class T>
  void setPGains(const T &pgain) {
    RSFATAL_IF(pgain.rows() !=dof, "p gains should have the same dimension as the degrees of freedom")
    kp_ = pgain;
    if(jointType[0] == Joint::FLOATING) {
      for(size_t i=0; i < 6; i++)
        kp_[i] = 0;
    }
    dampedDiagonalTermUpdated_ = false;
  }

  /**
   * set D gains.
   * @param[in] dgain velocity gain (dimension == getDOF())*/
  template<class T>
  void setDGains(const T &dgain) {
    RSFATAL_IF(dgain.rows() != dof, "d gains should have the same dimension as the degrees of freedom")
    kd_ = dgain;
    if(jointType[0] == Joint::FLOATING) {
      for(size_t i=0; i < 6; i++)
        kd_[i] = 0;
    }
    dampedDiagonalTermUpdated_ = false;
  }

  /**
   * passive elements at the joints. They can be specified in the URDF file as well. Check Object/ArticulatedSystem/URDF convention in the manual
   * @param[in] dampingCoefficient the damping coefficient vector, acting at each degrees of freedom */
  void setJointDamping(const Eigen::VectorXd &dampingCoefficient) {
    cd_ = dampingCoefficient;
    dampedDiagonalTermUpdated_ = false;
  }

  /**
   * passive elements at the joints. They can be specified in the URDF file as well. Check Object/ArticulatedSystem/URDF convention in the manual
   * @param[in] dampingCoefficient the damping coefficient vector, acting at each degrees of freedom */
  void setJointDamping(const raisim::VecDyn &dampingCoefficient) {
    cd_ = dampingCoefficient;
    dampedDiagonalTermUpdated_ = false;
  }

  /**
   * This computes the inverse mass matrix given the mass matrix. The return type is dense.
   * It exploits the sparsity of the mass matrix to efficiently perform the computation.
   * The outcome also contains effects of the joint damping and strings
   * @param[in] M mass matrix
   * @param[out] Minv inverse mass matrix */
  void computeSparseInverse(const MatDyn &M, MatDyn &Minv);

  /**
   * this method exploits the sparsity of the mass matrix. If the mass matrix is
   * nearly dense, it will be slower than your ordinary matrix multiplication
   * which is probably vectorized
   * vec = M * vec1
   * @param[in] vec input vector
   * @param[out] vec1 output vector*/
  inline void massMatrixVecMul(const VecDyn &vec1, VecDyn &vec) const;

  /**
   * The bodies specified here will not collide
   * @param[in] bodyIdx1 first body index
   * @param[in] bodyIdx2 second body index */
  void ignoreCollisionBetween(size_t bodyIdx1, size_t bodyIdx2);

  /**
   * Currently only supports "DO_NOT_COLLIDE_WITH_PARENT"
   * @return articulated system option */
  ArticulatedSystemOption getOptions() { return options_; }

  /**
   * @return a vector of body names (following the joint order) */
  const std::vector<std::string> &getBodyNames() { return bodyName; }

  /**
   * @return a vector of visualized bodies */
  std::vector<VisObject> &getVisOb() { return visObj; };

  /**
   * @return a vector of visualized collision bodies */
  std::vector<VisObject> &getVisColOb() { return visColObj; };

  /**
   * @param[in] visObjIdx visual object index. Following the order specified by the vector getVisOb()
   * @param[out] rot orientation
   * @param[out] pos position */
  void getVisObPose(size_t visObjIdx, Mat<3,3>& rot, Vec<3>& pos) { getPose(visObj[visObjIdx], rot, pos); }

  /**
   * @param[in] visColObjIdx visual collision object index. Following the order specified by the vector getVisColOb()
   * @param[out] rot orientation
   * @param[out] pos position */
  void getVisColObPose(size_t visColObjIdx, Mat<3,3>& rot, Vec<3>& pos) { getPose(visColObj[visColObjIdx], rot, pos); }

  /**
   * @return the resource directory (for mesh files, textures, etc) */
  const std::string &getResourceDir() { return resourceDir_; }

  /**
   * @return the resource directory (for mesh files, textures, etc) */
  const std::string &getRobotDescriptionfFileName() { return robotDefFileName_; }

  /**
   * @return the name of the URDF file (returns empty string if the robot was not specified by a URDF file) */
  const std::string &getRobotDescriptionfTopDirName() { return robotDefFileUpperDir_; }

  /**
   * @return the full path to the URDF file (returns empty string if the robot was not specified by a URDF file) */
  const std::string &getRobotDescriptionFullPath() { return fullURDFPath_; }

  /**
   * @return if the object was instantiated with raw URDF string, it returns the string */
  const std::string &getRobotDescription() { return robotDef_; }

  /**
   * if the object was instantiated with raw URDF string, it exports the robot description to an URDF file
   * @param[in] filePath Path where the file is generated */
  void exportRobotDescriptionToURDF(const std::string& filePath) {
    RSFATAL_IF(robotDef_.empty(), "This articulated system was created with an existing urdf file")
    std::ofstream urdfFile;
    RSFATAL_IF(filePath.substr(filePath.size()-5, 5)!=".urdf", "Articulated System is only saved to a URDF format")
    urdfFile.open (filePath);
    urdfFile << robotDef_;
    urdfFile.close();
  }

  /**
   * set the base position using an eigen vector
   * @param[in] pos position of the base */
  void setBasePos_e(const Eigen::Vector3d& pos) {
    if(jointType[0]==Joint::FIXED)
      jointPos_W[0].e() = pos;
    else {
      gc_[0] = pos[0]; gc_[1] = pos[1]; gc_[2] = pos[2];
    }
    updateKinematics();
  }

  /**
   * set the base orientation using an eigen vector
   * @param[in] rot orientation of the base */
  void setBaseOrientation_e(const Eigen::Matrix3d& rot) {
    if(jointType[0]==Joint::FIXED)
      rot_WB[0].e() = rot;
    else {
      Mat<3,3> rotMat{};
      Vec<4> quat;
      rotMat.e() = rot;
      rotMatToQuat(rotMat, quat);
      gc_.fillSegment(quat, 3);
    }
    updateKinematics();
  }

  /**
   * set the base position using an eigen vector
   * @param[in] pos position of the base */
  void setBasePos(const Vec<3>& pos);
  /**
   * set the base orientation using an eigen vector
   * @param[in] rot orientation of the base */
  void setBaseOrientation(const Mat<3,3>& rot);

  /**
   * set limits in actuation force. It can be also specified in the URDF file
   * @param[in] upper upper joint force/torque limit
   * @param[in] lower lower joint force/torque limit*/
  void setActuationLimits(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    tauUpper_ = upper; tauLower_ = lower;
  }

  /**
   * @return the upper joint torque/force limit*/
  const VecDyn& getActuationUpperLimits() { return tauUpper_; }

  /**
   * @return the lower joint torque/force limit*/
  const VecDyn& getActuationLowerLimits() { return tauLower_; }

  /**
   * change collision geom parameters.
   * @param[in] id collision object id
   * @param[in] params collision object parameters (depending on the object).
   * For a sphere, {raidus}.
   * For a cylinder and a capsule, {radius, length}
   * For a box, {x-dim, y-dim, z-dim} */
  void setCollisionObjectShapeParameters(size_t id, const std::vector<double>& params);

  /**
   * change collision geom offset from the joint position.
   * @param[in] id collision object id
   * @param[in] posOffset the position vector expressed in the joint frame */
  void setCollisionObjectPositionOffset(size_t id, const Vec<3>& posOffset);

  /**
   * change collision geom orientation offset from the joint frame.
   * @param[in] id collision object id
   * @param[in] oriOffset the orientation relative to the joint frame */
  void setCollisionObjectOrientationOffset(size_t id, const Mat<3,3>& oriOffset);

  /**
   * rotor inertia is a term added to the diagonal of the mass matrix. This approximates the rotor inertia. Note that this
   * is not exactly equivalent in dynamics (due to gyroscopic effect). but it is a commonly used approximation.
   * It can also be expressed in the URDF file
   * @param[in] rotorInertia the rotor inertia */
  void setRotorInertia(const VecDyn& rotorInertia) {
    rotorInertia_ = rotorInertia;
  }

  /**
   * rotor inertia is a term added to the diagonal of the mass matrix. This approximates the rotor inertia. Note that this
   * is not exactly equivalent in dynamics (due to gyroscopic effect). but it is a commonly used approximation.
   * It can also be expressed in the URDF file
   * @return the rotor inertia */
  const VecDyn& getRotorInertia() {
    return rotorInertia_;
  }

  /**
   * This joint indices are in the same order as the elements of the generalized velocity
   * However, some joints have multiple degrees of freedom and they are not equal
   * @param[in] jointIndex the joint index
   * @return the joint type */
  Joint::Type getJointType(size_t jointIndex) {
    return jointType[jointIndex];
  }

  /**
   * @return the number of joints (same as the number of bodies) */
  size_t getNumberOfJoints() const {
    return nbody;
  }

  /**
   * returns reference object of the joint
   * @param[in] name joint name
   * @return a JointRef to the joint. Check the example JointRefAndLinkRef*/
  JointRef getJoint(const std::string& name) {
    auto index = getFrameIdxByName(name);
    return {index, this};
  }

  /**
   * returns reference object of the joint
   * @param[in] name the link name
   * @return a LinkRef to the link. Check the example JointRefAndLinkRef*/
  LinkRef getLink(const std::string& name) {
    return {size_t(std::find(bodyName.begin(), bodyName.end(), name)-bodyName.begin()), this};
  }

  /**
   * @return a mapping that converts body index to gv index */
  const std::vector<size_t>& getMappingFromBodyIndexToGeneralizedVelocityIndex() const {
    return bodyIdx2GvIdx;
  }

  /**
   * @return a mapping that converts body index to gv index */
  const std::vector<size_t>& getMappingFromBodyIndexToGeneralizedCoordinateIndex() const {
    return bodyIdx2GcIdx;
  }

  /**
   * @return the object type (ARTICULATED_SYSTEM) */
  ObjectType getObjectType() const final { return ARTICULATED_SYSTEM; }

  /**
   * @return the body type (STATIC, KINEMATIC, or DYNAMIC) of the specified body. It is always DYNAMIC except for the fixed base*/
  BodyType getBodyType(size_t bodyIdx) const final;

  /**
   * @return the body type (STATIC, KINEMATIC, or DYNAMIC). It is always dynamic */
  BodyType getBodyType() const final { return BodyType::DYNAMIC; };

  /**
   * @param[in] scheme the integration scheme. Can be either TRAPEZOID, SEMI_IMPLICIT, EULER, or RUNGE_KUTTA_4. We recommend TRAPEZOID for systems with many collisions.
   * RUNGE_KUTTA_4 is useful for systems with few contacts and when the integration accuracy is important */
  void setIntegrationScheme(IntegrationScheme scheme) {
    desiredIntegrationScheme_ = scheme;
    computeDampedMass(dt_);
    dampedDiagonalTermUpdated_ = true;
  }

  /**
   * usage example:
   * For 1d joints (e.g., revolute or prismatic), you can get the impulse due to the joint limit as
   * ``robot.getJointLimitViolations()[0]->imp_i[0]``
   * For a ball joint, all three components of imp_i represent the torque in the 3d space.
   * The following joint returns the joint/body Id
   * ``robot.getJointLimitViolations()[0]->jointId``.
   * @return get contact problems associated with violated joint limits */
  const std::vector<contact::Single3DContactProblem*>& getJointLimitViolations() {
    return jointLimitViolation_;
  }

 protected:

  void getPose(const VisObject& vob, Mat<3, 3>& rot, Vec<3>& pos);

  void getSparseJacobian_internal(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  void updateCollision() final;

  void computeMassMatrix(MatDyn &M);

  void computeNonlinearities(const Vec<3> &gravity, VecDyn &b);

  void destroyCollisionBodies(dSpaceID id) final;

  /* computing JM^-1J^T exploiting sparsity */
  void getFullDelassusAndTauStar(double dt);

  /* This computes Delassus matrix necessary for contact force computation */
  void preContactSolverUpdate1(const Vec<3> &gravity, double dt) final;
  void preContactSolverUpdate2(const Vec<3> &gravity, double dt) final;
  void integrateWithoutContact(const Vec<3> &gravity, double dt);

  void integrate(double dt, const Vec<3>& gravity) final;

  void addContactPointVel(size_t pointId, Vec<3> &vel) final;

  void subContactPointVel(size_t pointId, Vec<3> &vel) final;

  void updateGenVelWithImpulse(size_t pointId, const Vec<3> &imp) final;

  void updateTimeStep(double dt) final;

  void updateTimeStepIfNecessary(double dt) final;

  /* adding spring element */
  void addSpring(const SpringElement& spring) { springs_.push_back(spring); }

  std::vector<SpringElement>& getSprings() { return springs_; }
  const std::vector<SpringElement>& getSprings() const { return springs_; }

  inline void jacoSub(const raisim::SparseJacobian &jaco1, raisim::SparseJacobian &jaco, bool isFloatingBase);

  void setConstraintForce(size_t bodyIdx, const Vec<3>& pos, const Vec<3>& force) final;

 private:

  void init();

  void integratePosition(VecDyn& gc, VecDyn& gvAvg, double dt);

  void computeExpandedParentArray();

  void appendJointLimits(contact::ContactProblems &problem) final;

  double enforceJointLimits(contact::Single3DContactProblem &problem) final;

  void computeDampedMass(double dt);

  void cleanContacts() { contactJaco_.clear(); getContacts().clear(); }

  void rkIntegrate(const Vec<3>& gravity, double dt);

  void clearExternalForces();

  /// for computation
  /// Frames:: W: world, B: body, P: parent

  /// Configuration Variables
  std::vector<raisim::Mat<3, 3>> rot_WB;
  std::vector<raisim::Mat<3, 3>> rot_JC;
  std::vector<raisim::Vec<3>> jointAxis_W;
  std::vector<raisim::Vec<3>> jointPos_W;
  std::vector<raisim::Vec<3>> joint2com_W;
  std::vector<raisim::Vec<3>> comPos_W;
  std::vector<raisim::Mat<3, 3>> compositeInertia_W;
  std::vector<raisim::Vec<3>> composite_mXCOM_W;
  std::vector<raisim::Vec<3>> composite_com_W;
  std::vector<raisim::Vec<3>> bodyLinearVel_W;
  std::vector<raisim::Vec<3>> bodyAngVel_W, bodyAngVel_B;
  std::vector<raisim::Vec<3>> bodyAngVel_pc_W;
  std::vector<raisim::Vec<3>> bodyLinVel_pc_W;
  std::vector<raisim::Vec<3>> bodyLinearAcc;
  std::vector<raisim::Vec<3>> bodyAngAcc;
  std::vector<raisim::Vec<3>> propagatingForceAtJoint_W;
  std::vector<raisim::Vec<3>> propagatingTorqueAtJoint_W;
  std::vector<raisim::Mat<3, 3>> inertiaAboutJoint_W;
  std::vector<raisim::Mat<3, 3>> inertia_comW;
  std::vector<raisim::Vec<3>> sketch;
  std::vector<raisim::Vec<3>> joint2joint_W;

  /// Constant parameters of the robot
  std::vector<raisim::Mat<3, 3>> rot_JB, rot_WJ;
  std::vector<raisim::Vec<3>> jointPos_P;
  std::vector<raisim::Vec<3>> jointAxis_P;
  std::vector<raisim::Mat<3, 3>> inertia_comB;
  std::vector<raisim::Vec<3>> comPos_B;
  std::vector<raisim::Vec<2>> jointLimits_;
  Vec<3> fixedBasePos_;

  std::vector<Joint::Type> jointType;
  std::vector<size_t> jointGcDim_;
  std::vector<size_t> jointGvDim_;

  std::vector<double> mass;
  std::vector<size_t> parent;
  std::vector<size_t> leafnodes_;
  std::vector<double> compositeMass;

  std::vector<std::vector<size_t> > children_;
  std::vector<size_t> toBaseBodyCount_; // counts to base (which is WORLD).
  std::vector<size_t> toBaseGvDimCount_;
  std::vector<size_t> toBaseGcDimCount_;
  std::vector<size_t> lambda;

  /// State variables
  VecDyn gc_, gcOld_, gv_, gvOld_, gvAvg_, h_, gvERP_, gvTemp_, gvPreimpact_, gcRK_[4], gvRK_[4], slopeRK_[4];
  MatDyn M_, Minv_, lT_;

  /// Contact variables
  std::vector<MatDyn> MinvJT_T;
  VecDyn tauStar_, tau_, tauFF_; // tau_ is the actual torque applied to the joints. tauFF_ is the feedforward joint torque. these two can be differ if the control mode is not FORCE_AND_TORQUE
  VecDyn tauUpper_, tauLower_; // bounds
  std::vector<size_t> bodyIdx2GvIdx, bodyIdx2GcIdx;

  std::vector<SparseJacobian> J_;

  std::vector<CoordinateFrame> frameOfInterest_;

  /// total linear momentum in cartesian space
  Vec<3> linearMomentum_;

 protected:
  std::vector<Child> rootChild_;
  std::vector<std::string> bodyName;
  std::vector<std::string> movableJointNames;

  std::vector<SparseJacobian> contactJaco_;
  std::vector<SparseJacobian> externalForceAndTorqueJaco_;

  raisim::CollisionSet collisionBodies;
  std::vector<VisObject> visColObj, visObj;
  ArticulatedSystemOption options_;
  std::vector<contact::Single3DContactProblem*> jointLimitViolation_;

 private:
  size_t nbody, dof = 0, gcDim = 0, baseDOFminusOne = 0;
  bool hasSphericalJoint_ = false;
  bool kinematicsUpdated_ = false;
  bool dampedDiagonalTermUpdated_ = false;
  ControlMode::Type controlMode_ = ControlMode::FORCE_AND_TORQUE;
  raisim::SparseJacobian tempJaco_;
  Vec<3> gravity_;
  double dt_ = 0.001;

  // damping
  VecDyn cd_;
  VecDyn cf_;

  std::vector<SpringElement> springs_;

  // stable PD controller
  VecDyn kp_, kd_, uref_, qref_, diag_w, posErr_, uErr_;
  VecDyn rotorInertia_;
  VecDyn generalizedMomentum_;
  VecDyn temp, temp2;
  MatDyn Mtemp_;
  std::string resourceDir_, robotDefFileName_, robotDefFileUpperDir_, fullURDFPath_, robotDef_;

  // for Trimesh
  std::vector<dTriMeshDataID> meshData_;
  std::vector<std::vector<float>> meshVertices_;
  std::vector<std::vector<dTriIndex>> meshIdx_;

  IntegrationScheme desiredIntegrationScheme_ = IntegrationScheme::TRAPEZOID;
  IntegrationScheme integrationSchemeThisTime_;
};

}

#endif //RAISIM_ARTICULATEDSYSTEM_HPP
