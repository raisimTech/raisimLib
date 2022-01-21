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
#include "raisim/contact/BisectionContactSolver.hpp"
#include "raisim/constraints/PinConstraint.hpp"

namespace raisim {

namespace mjcf {
class LoadFromMjcf;
struct MjcfCompilerSetting;
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

  bool isSecondOrderOrHigher(IntegrationScheme scheme) const { return int(scheme) > int(IntegrationScheme::EULER); }

  struct SpringElement {
    SpringElement() { q_ref.setZero(); }

    void setSpringMount (const Eigen::Vector4d& qRef) { q_ref = qRef; }
    Eigen::Vector4d getSpringMount () { return q_ref.e(); }

    /// the spring connects this body and its parent
    size_t childBodyId = 0;
    /// spring stiffness
    double stiffness = 0;
    /// mounting angles for torsional and linear springs
    Vec<4> q_ref;
  };

  class LinkRef {
   public:
    LinkRef(size_t localId, ArticulatedSystem *system) :
        system_(system), localId_(localId) {

      for (auto &col: system_->getCollisionBodies()) {
        if (col.localIdx == localId_) {
          colDef_.insert({col.colObj->name, &col});
        }
      }

      for (auto &vis: system_->visObj) {
        if (vis.localIdx == localId_) {
          visDef_.insert({vis.name, &vis});
        }
      }
    }

    void getPosition(Vec<3> &position) const {
      system_->getBodyPosition(localId_, position);
    }

    void getOrientation(Mat<3, 3> &orientation) const {
      system_->getBodyOrientation(localId_, orientation);
    }

    void getPose(Vec<3> &position, Mat<3, 3> &orientation) const {
      system_->getBodyPose(localId_, orientation, position);
    }

    CollisionDefinition *getCollisionDefinition(const std::string &name) {
      return colDef_[name];
    }

    VisObject *getVisualObject(const std::string &name) {
      return visDef_[name];
    }

    void setWeight(double weight) {
      system_->getMass()[localId_] = weight;
      system_->updateMassInfo();
    }

    double getWeight() const {
      return system_->getMass(localId_);
    }

    void setInertia(const Mat<3, 3> &inertia) {
      system_->getInertia()[localId_] = inertia;
    }

    const Mat<3, 3> &getInertia() const {
      return system_->getInertia()[localId_];
    }

    /* set center of mass position in parent frame */
    void setComPositionInParentFrame(const Vec<3> &com) {
      system_->getBodyCOM_B()[localId_] = com;
    }

    /* get center of mass position in parent frame */
    const Vec<3> &getComPositionInParentFrame() const {
      return system_->getBodyCOM_B()[localId_];
    }

    const std::unordered_map<std::string, CollisionDefinition *> &getCollisionSet() const {
      return colDef_;
    }

    const std::unordered_map<std::string, VisObject *> &getVisualSet() const {
      return visDef_;
    }

   private:
    raisim::ArticulatedSystem *system_;
    size_t localId_;
    std::unordered_map<std::string, CollisionDefinition *> colDef_;
    std::unordered_map<std::string, VisObject *> visDef_;
  };

  class JointRef {
   public:
    JointRef(size_t frameId, ArticulatedSystem *system) :
        system_(system), frameId_(frameId) {
      auto &toGvIndex = system_->getMappingFromBodyIndexToGeneralizedVelocityIndex();
      auto &toGcIndex = system_->getMappingFromBodyIndexToGeneralizedCoordinateIndex();

      isMovable_ = system_->getFrames()[frameId].isChild;

      if (isMovable_) {
        gvIndx_ = toGvIndex[system_->getFrames()[frameId].parentId];
        gcIndx_ = toGcIndex[system_->getFrames()[frameId].parentId];
      } else {
        gvIndx_ = -1;
        gcIndx_ = -1;
      }

      jointType_ = system_->getFrames()[frameId].jointType;
    }

    void getPosition(Vec<3> &position) const {
      system_->getFramePosition(frameId_, position);
    }

    Mat<3, 3> getOrientation() const {
      Mat<3, 3> orientation;
      system_->getFrameOrientation(frameId_, orientation);
      return orientation;
    }

    void getPose(Vec<3> &position, Mat<3, 3> &orientation) const {
      system_->getFramePosition(frameId_, position);
      system_->getFrameOrientation(frameId_, orientation);
    }

    /* Joint coordinate can be multiple dimensional vector (for a spherical joint)
     * or 1D vector */
    void getJointCoordinate(VecDyn &coordinate) const {
      if (jointType_ == raisim::Joint::Type::SPHERICAL) {
        coordinate.resize(4);
        coordinate[0] = system_->getGeneralizedCoordinate()[gcIndx_];
        coordinate[1] = system_->getGeneralizedCoordinate()[gcIndx_ + 1];
        coordinate[2] = system_->getGeneralizedCoordinate()[gcIndx_ + 2];
        coordinate[3] = system_->getGeneralizedCoordinate()[gcIndx_ + 3];
      }
      coordinate[0] = system_->getGeneralizedCoordinate()[gcIndx_];
    }

    double getJointAngle() const {
      return system_->getGeneralizedCoordinate()[gcIndx_];
    }

    const raisim::Vec<3> &getPositionInParentFrame() const {
      RSFATAL_IF(!isMovable_, "This is a fixed joint. You cannot change the position of a fixed joint.")
      return system_->getJointPos_P()[gvIndx_];
    }

    const raisim::Vec<3> &getJointAxis() const {
      RSFATAL_IF(!isMovable_, "This is a fixed joint. You cannot change the position of a fixed joint.")
      return system_->getJointAxis_P()[gvIndx_];
    }

    Joint::Type getType() const {
      return system_->getJointType(gvIndx_);
    }

    size_t getIdxInGeneralizedCoordinate() const {
      return gvIndx_;
    }

    Vec<3> getLinearVelocity() const {
      Vec<3> linVel;
      system_->getFrameVelocity(frameId_, linVel);
      return linVel;
    }

   private:
    raisim::ArticulatedSystem *system_;
    size_t frameId_, gvIndx_, gcIndx_;
    bool isMovable_;
    Joint::Type jointType_;
  };

 private:
  friend class raisim::World;
  friend class raisim::urdf::LoadFromURDF2;
  friend class raisim::mjcf::LoadFromMjcf;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ArticulatedSystem() = default;

  ArticulatedSystem(const Child &child,
                    const std::string &resDir,
                    ArticulatedSystemOption options);

  explicit ArticulatedSystem(const std::string &filePathOrURDFScript,
                             const std::string &resDir = "",
                             const std::vector<std::string> &jointOrder = std::vector<std::string>(),
                             ArticulatedSystemOption options = ArticulatedSystemOption());

  ArticulatedSystem(const RaiSimTinyXmlWrapper &c,
                    const std::string &resDir,
                    const std::unordered_map<std::string, RaiSimTinyXmlWrapper>& defaultNode,
                    const std::unordered_map<std::string, std::pair<std::string, Vec<3>>>& mesh,
                    const mjcf::MjcfCompilerSetting& setting,
                    ArticulatedSystemOption options = ArticulatedSystemOption());

  ~ArticulatedSystem();

  /**
   * @return generalized coordinate of the system */
  const raisim::VecDyn &getGeneralizedCoordinate() const { return gc_; }

  /**
   * @return generalized velocity of the system */
  const raisim::VecDyn &getGeneralizedVelocity() const { return gv_; }

  /**
   * @param[out] quaternion orientation of base*/
  void getBaseOrientation(raisim::Vec<4> &quaternion) const;

  /**
   * @param[out] rotataionMatrix orientation of base*/
  void getBaseOrientation(raisim::Mat<3, 3> &rotataionMatrix) const { rotataionMatrix = rot_WB[0]; }

  /**
   * @return orientation of base*/
  const raisim::Mat<3, 3> &getBaseOrientation() const { return rot_WB[0]; }

  /**
   * @param[out] position position of base*/
  void getBasePosition(raisim::Vec<3> &position) const {
    RSFATAL_IF(jointType[0] != Joint::Type::FLOATING, "This method is only for floating base")
    position[0] = gc_[0];
    position[1] = gc_[1];
    position[2] = gc_[2];
  }

  /**
   * @return position of base*/
  raisim::Vec<3> getBasePosition() const {
    RSFATAL_IF(jointType[0] != Joint::Type::FLOATING, "This method is only for floating base")
    raisim::Vec<3> position;
    position[0] = gc_[0];
    position[1] = gc_[1];
    position[2] = gc_[2];
    return position;
  }

  /**
   * unnecessary to call this function if you are simulating your system. integrate1 calls this function
   * Call this function if you want to get kinematic properties but you don't want to integrate.  */
  void updateKinematics();

  /**
   * set gc of each joint in order
   * @param[in] jointState generalized coordinate */
  void setGeneralizedCoordinate(const Eigen::VectorXd &jointState) {
    gc_ = jointState;
    updateKinematics();
    cleanContacts();
  }
  /**
   * set gc of each joint in order
   * @param[in] jointState generalized coordinate */
  void setGeneralizedCoordinate(const raisim::VecDyn &jointState) {
    gc_ = jointState;
    updateKinematics();
    cleanContacts();
  }

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
   * @param[in] jointVel the generalized velocity */
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
  void getState(Eigen::VectorXd &genco, Eigen::VectorXd &genvel) const {
    genco = gc_.e();
    genvel = gv_.e();
  }

  /**
   * get both the generalized coordinate and the generalized velocity
   * @param[out] genco the generalized coordinate
   * @param[out] genvel the generalized velocity */
  void getState(VecDyn &genco, VecDyn &genvel) const {
    genco = gc_;
    genvel = gv_;
  }

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
   * Generalized user-set gen force (using setGeneralizedForce()).
   * This method a small error when the built-in PD controller is used.
   * The PD controller is implicit (using a continuous, linear model) so we cannot get the true gen force.
   * But if you set the time step small enough, the difference is negligible.
   * @return the generalized force */
  VecDyn getGeneralizedForce() const {
    VecDyn genForce(dof);
    genForce = tauFF_;
    if (controlMode_ == ControlMode::PD_PLUS_FEEDFORWARD_TORQUE) {
      vecvecCwiseMulThenAdd(kp_, posErr_, genForce);
      vecvecCwiseMulThenSub(kd_, gv_, genForce);
      vecvecCwiseMulThenAdd(kd_, uref_, genForce);
    }
    return genForce;
  }

  /**
   * get the feedfoward generalized force (which is set by the user)
   * @return the feedforward generalized force */
  const VecDyn &getFeedForwardGeneralizedForce() const { return tauFF_; }

  /**
   * get the mass matrix
   * @return the mass matrix. Check Object/ArticulatedSystem section in the manual */
  const MatDyn &getMassMatrix() { computeMassMatrix(M_); return M_; }

  /**
   * get the coriolis and the gravitational term
   * @param[in] gravity gravitational acceleration. You should get this value from the world.getGravity();
   * @return the coriolis and the gravitational term. Check Object/ArticulatedSystem section in the manual */
  const VecDyn &getNonlinearities(const Vec<3>& gravity) { computeNonlinearities(gravity, h_); return h_; }

  /**
   * get the inverse mass matrix. Note that this is actually damped inverse.
   * It contains the effect of damping and the spring effects due to the implicit integration.
   * YOU MUST CALL getMassMatrix FIRST BEFORE CALLING THIS METHOD.
   * @return the inverse mass matrix. Check Object/ArticulatedSystem section in the manual */
  const MatDyn &getInverseMassMatrix() { computeSparseInverse(M_, Minv_); return Minv_; }

  /**
   * get the center of mass of a composite body containing body i and all its children.
   * if you want the COM of the whole robot, just take the first element
   * This only works if you have called getMassMatrix() with the current state
   * @return the center of mass of the composite body */
  const std::vector<raisim::Vec<3>> &getCompositeCOM() const { return composite_com_W; }

  /**
   * get the center of mass of the whole system
   * @return the center of mass of the system */
  Vec<3> getCOM() const {
    Vec<3> com = {0,0,0};
    double massTotal = 0;
    for(size_t i=0; i<nbody; i++) {
      massTotal += mass[i];
      com += mass[i] * comPos_W[i];
    }

    com *= 1./massTotal;
    return com;
  }

  /**
   * get the current composite inertia of a composite body containing body i and all its children
   * @return the inertia of the composite system */
  const std::vector<raisim::Mat<3, 3>> &getCompositeInertia() const { return compositeInertia_W; }

  /**
   * get the current composite mass of a composite body containing body i and all its children
   * @return get the composite mass */
  const std::vector<double> &getCompositeMass() const { return compositeMass; }

  /**
   * linear momentum of the whole system
   * @return momentum */
  Vec<3> getLinearMomentum() const {
    Vec<3> linearMomentum;
    linearMomentum.setZero();

    for (size_t i = 0; i < nbody; i++) {
      Vec<3> com_vel;
      ArticulatedSystem::getVelocity(i, comPos_B[i], com_vel);
      vecScalarMul(mass[i], com_vel);
      vecadd(com_vel, linearMomentum);
    }
    return linearMomentum;
  }

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
  double getPotentialEnergy(const Vec<3> &gravity) const;

  /**
   * @param[in] referencePoint the reference point about which the angular momentum is computed
   * @param[in] angularMomentum angular momentum about the reference point*/
  void getAngularMomentum(const Vec<3> &referencePoint, Vec<3> &angularMomentum) const;

  /**
   * bodies here means moving bodies. Fixed bodies are optimized out*/
  void printOutBodyNamesInOrder() const;

  /**
   * print out movable joint names in order */
  void printOutMovableJointNamesInOrder() const;

  /**
   * frames are attached to every joint coordinate */
  void printOutFrameNamesInOrder() const;

  /**
   * getMovableJointNames. Note! the order doesn't correspond to dof since there are
   * joints with multiple dof's
   * @return movable joint names in the joint order. */
  const std::vector<std::string> &getMovableJointNames() const { return movableJointNames; };

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
  CoordinateFrame &getFrameByName(const std::string &nm) { return frameOfInterest_[getFrameIdxByName(nm)]; }
  const CoordinateFrame &getFrameByName(const std::string &nm) const { return frameOfInterest_[getFrameIdxByName(nm)]; }

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * @param[in] name name of the urdf link that is a child of the joint
   * @return the coordinate frame of the given link name */
  CoordinateFrame &getFrameByLinkName(const std::string &name) {
    return *std::find_if(frameOfInterest_.begin(), frameOfInterest_.end(),
                        [name](const raisim::CoordinateFrame &ref) { return ref.bodyName == name; });
  }
  const CoordinateFrame &getFrameByLinkName(const std::string &name) const {
    return *std::find_if(frameOfInterest_.begin(), frameOfInterest_.end(),
                         [name](const raisim::CoordinateFrame &ref) { return ref.bodyName == name; });
  }

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * @param[in] name name of the urdf link that is a child of the joint
   * @return the coordinate frame index of the given link name */
  size_t getFrameIdxByLinkName(const std::string &name) const {
    return std::find_if(frameOfInterest_.begin(), frameOfInterest_.end(),
                        [name](const raisim::CoordinateFrame &ref) { return ref.bodyName == name; })
                        - frameOfInterest_.begin();
  }

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * @param[in] idx index of the frame
   * @return the coordinate frame of the given index */
  CoordinateFrame &getFrameByIdx(size_t idx) { return frameOfInterest_[idx]; }
  const CoordinateFrame &getFrameByIdx(size_t idx) const { return frameOfInterest_[idx]; }

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * The frame can be retrieved as as->getFrames[index]. This way is more efficient than above methods that use the frame name
   * @param[in] nm name of the frame
   * @return the index of the coordinate frame of the given index. Returns size_t(-1) if it doesn't exist */
  size_t getFrameIdxByName(const std::string &nm) const;

  /**
   * Refer to Object/ArticulatedSystem/Kinematics/Frame in the manual for details
   * The frame can be retrieved as as->getFrames[index]. This way is more efficient than above methods that use the frame name
   * @return a vector of the coordinate frames */
  std::vector<CoordinateFrame> &getFrames() { return frameOfInterest_; };
  const std::vector<CoordinateFrame> &getFrames() const { return frameOfInterest_; };

  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[out] point_W the position of the frame expressed in the world frame */
  void getFramePosition(size_t frameId, Vec<3> &point_W) const;


  /**
   * @param[in] frameId the frame id which can be obtained by getFrameIdxByName()
   * @param[in] localPos local position in the specified frame
   * @param[out] point_W the position expressed in the world frame */
  void getPositionInFrame(size_t frameId, const Vec<3> &localPos, Vec<3> &point_W) const;

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
  void getFramePosition(const std::string &frameName, Vec<3> &point_W) const {
    getFramePosition(getFrameIdxByName(frameName), point_W);
  }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] orientation_W the orientation of the frame relative to the world frame */
  void getFrameOrientation(const std::string &frameName, Mat<3, 3> &orientation_W) const {
    getFrameOrientation(getFrameIdxByName(frameName), orientation_W);
  }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] vel_W the linear velocity of the frame expressed in the world frame */
  void getFrameVelocity(const std::string &frameName, Vec<3> &vel_W) const {
    getFrameVelocity(getFrameIdxByName(frameName), vel_W);
  }

  /**
   * @param[in] frameName the frame name (defined in the urdf)
   * @param[out] angVel_W the angular velocity of the frame relative to the world frame */
  void getFrameAngularVelocity(const std::string &frameName, Vec<3> &angVel_W) const {
    getFrameAngularVelocity(getFrameIdxByName(frameName), angVel_W);
  }

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] point_W the position of the frame relative to the world frame */
  void getFramePosition(const CoordinateFrame &frame, Vec<3> &point_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] orientation_W the orientation of the frame relative to the world frame */
  void getFrameOrientation(const CoordinateFrame &frame, Mat<3, 3> &orientation_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] vel_W the linear velocity of the frame expressed to the world frame */
  void getFrameVelocity(const CoordinateFrame &frame, Vec<3> &vel_W) const;

  /**
   * @param[in] frame custom frame defined by the user
   * @param[out] angVel_W the angular velocity of the frame expressed to the world frame */
  void getFrameAngularVelocity(const CoordinateFrame &frame, Vec<3> &angVel_W) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] pos_w the position of the joint (after its own joint transformation) */
  void getPosition(size_t bodyIdx, Vec<3> &pos_w) const final { pos_w = jointPos_W[bodyIdx]; }

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[in] pos_W the position in the world coordinate. This position does not have to be physically on the body.
   * @param[out] pos_B the position in the body frame */
  void getPositionInBodyCoordinate(size_t bodyIdx, const Vec<3>& pos_W, Vec<3>& pos_B);

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
  void getSparseJacobian(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[in] frame the frame in which the position of the point is expressed in
   * @param[in] point the point expressed in the world frame. If you want to use a point expressed in the body frame, use getDenseFrameJacobian()
   * @param[out] jaco the positional Jacobian. v = J * u. v is the linear velocity expressed in the world frame and u is the generalized velocity */
  void getSparseJacobian(size_t bodyIdx, Frame frame, const Vec<3> &point, SparseJacobian &jaco) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] jaco the rotational Jacobian. omega = J * u. omgea is the angular velocity expressed in the world frame and u is the generalized velocity */
  void getSparseRotationalJacobian(size_t bodyIdx, SparseJacobian &jaco) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[in] frame the frame in which the position of the point is expressed
   * @param[in] point the position of the point of interest
   * @param[out] jaco the time derivative of the positional Jacobian. a = dJ * u + J * du. a is the linear acceleration expressed in the world frame, u is the generalized velocity and d denotes the time derivative*/
  void getTimeDerivativeOfSparseJacobian(size_t bodyIdx, Frame frame, const Vec<3> &point, SparseJacobian &jaco) const;

  /**
   * @param[in] bodyIdx the body index. Note that body index and the joint index are the same because every body has one parent joint. It can be retrieved by getBodyIdx()
   * @param[out] jaco the rotational Jacobian. alpha = dJ * u + J * du. alpha is the angular acceleration expressed in the world frame, u is the generalized velocity and d denotes the time derivative*/
  void getTimeDerivativeOfSparseRotationalJacobian(size_t bodyIdx, SparseJacobian &jaco) const;

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
  void getDenseJacobian(size_t bodyIdx, const Vec<3> &point_W, Eigen::MatrixXd &jaco) const {
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
  void getDenseRotationalJacobian(size_t bodyIdx, Eigen::MatrixXd &jaco) const {
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
  void getDenseFrameJacobian(size_t frameIdx, Eigen::MatrixXd &jaco) const {
    auto &frame = getFrameByIdx(frameIdx);
    Vec<3> position_W;
    getFramePosition(frameIdx, position_W);
    getDenseJacobian(frame.parentId, position_W, jaco);
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameName the frame name. (defined in the URDF)
   * @param[out] jaco the dense positional Jacobian */
  void getDenseFrameJacobian(const std::string &frameName, Eigen::MatrixXd &jaco) const {
    getDenseFrameJacobian(getFrameIdxByName(frameName), jaco);
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameIdx the frame index. it can be retrieved by getFrameIdxByName()
   * @param[out] jaco the dense rotational Jacobian */
  void getDenseFrameRotationalJacobian(size_t frameIdx, Eigen::MatrixXd &jaco) const {
    auto &frame = getFrameByIdx(frameIdx);
    getDenseRotationalJacobian(frame.parentId, jaco);
  }

  /**
   * This method only fills out non-zero elements. Make sure that the jaco is setZero() once in the initialization!
   * @param[in] frameName the frame name. (defined in the URDF)
   * @param[out] jaco the dense rotational Jacobian */
  void getDenseFrameRotationalJacobian(const std::string &frameName, Eigen::MatrixXd &jaco) const {
    getDenseFrameRotationalJacobian(getFrameIdxByName(frameName), jaco);
  }

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
   * @param[in] frameOfPos the frame in which the provided position is expressed
   * @param[in] pos the position of the point of interest
   * @param[in] frameOfVel the frame in which the computed velocity is expressed
   * @param[out] pointVel the velocity of the point expressed in the world frame */
  void getVelocity(size_t bodyIdx, Frame frameOfPos, const Vec<3> &pos, Frame frameOfVel, Vec<3> &pointVel) const;

  /**
   * returns the index of the body
   * @param[in] nm name of the body. The body name is the name of the movable link of the body
   * @return the index of the body. Returns size_t(-1) if the body doesn't exist. */
  size_t getBodyIdx(const std::string &nm) const;

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
  void getBodyPose(size_t bodyIdx, Mat<3, 3> &orientation, Vec<3> &position) const {
    orientation = rot_WB[bodyIdx];
    position = jointPos_W[bodyIdx];
  }

  /**
   * The body position is the position of its parent joint (after its joint transformation)
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] position the position of the body */
  void getBodyPosition(size_t bodyIdx, Vec<3> &position) const { position = jointPos_W[bodyIdx]; }

  /**
   * The body orientation is the orientation of its parent joint (after its joint transformation)
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[out] orientation the orientation of the body */
  void getBodyOrientation(size_t bodyIdx, Mat<3, 3> &orientation) const { orientation = rot_WB[bodyIdx]; }

  /**
   * The following 5 methods can be used to directly modify dynamic/kinematic properties of the robot.
   * They are made for dynamic randomization. Use them with caution since they will change the
   * the model permenantly. After you change the dynamic properties, call "void updateMassInfo()" to update
   * some precomputed dynamic properties */
  /**
   * @return a reference to joint position relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Vec<3>> &getJointPos_P() { return jointPos_P; }
  const std::vector<raisim::Vec<3>> &getJointPos_P() const { return jointPos_P; }

  /**
   * @return a reference to joint orientation relative to its parent. */
  std::vector<raisim::Mat<3, 3>> &getJointOrientation_P() { return rot_JB; }
  const std::vector<raisim::Mat<3, 3>> &getJointOrientation_P() const { return rot_JB; }

  /**
   * @return a reference to joint axis relative to its parent, expressed in the parent frame. */
  std::vector<raisim::Vec<3>> &getJointAxis_P() { return jointAxis_P; }
  const std::vector<raisim::Vec<3>> &getJointAxis_P() const { return jointAxis_P; }

  /**
   * @return a reference to joint axis expressed in the world frame. */
  const raisim::Vec<3> &getJointAxis(size_t idx) const { return jointAxis_W[idx]; }

  /**
   * You MUST call updateMassInfo() after you change the mass
   * @return a reference to mass of each body.*/
  std::vector<double> &getMass() { return mass; }
  const std::vector<double> &getMass() const { return mass; }

  /**
   * @return a reference to inertia of each body.*/
  std::vector<raisim::Mat<3, 3>> &getInertia() { return inertia_comB; }
  const std::vector<raisim::Mat<3, 3>> &getInertia() const { return inertia_comB; }

  /**
   * @return a reference to the position of the center of the mass of each body in the body frame.*/
  std::vector<raisim::Vec<3>> &getBodyCOM_B() { return comPos_B; }
  const std::vector<raisim::Vec<3>> &getBodyCOM_B() const { return comPos_B; }

  /**
 * @return a reference to the position of the center of the mass of each body in the world frame.*/
  std::vector<raisim::Vec<3>> &getBodyCOM_W() { return comPos_W; }
  const std::vector<raisim::Vec<3>> &getBodyCOM_W() const { return comPos_W; }

  /**
   * @return a reference to the collision bodies. Position and orientation can be set dynamically */
  raisim::CollisionSet &getCollisionBodies() { return collisionBodies; }
  const raisim::CollisionSet &getCollisionBodies() const { return collisionBodies; }

  /**
   * @param name collision body name which is "LINK_NAME" + "/" + "COLLISION_NUMBER". For example, the first collision body of the link "base" is named as "base/0"
   * @return a reference to the collision bodies. Position and orientation can be set dynamically */
  raisim::CollisionDefinition &getCollisionBody(const std::string &name) {
    return *std::find_if(collisionBodies.begin(), collisionBodies.end(),
                         [name](const raisim::CollisionDefinition &ref) { return ref.colObj->name == name; });
  }
  const raisim::CollisionDefinition &getCollisionBody(const std::string &name) const {
    return *std::find_if(collisionBodies.begin(), collisionBodies.end(),
                         [name](const raisim::CollisionDefinition &ref) { return ref.colObj->name == name; });
  }

  /**
   * This method updates the precomputed composite mass. Call this method after you change link mass.
   * This also updates the center of mass without integration*/
  void updateMassInfo();

  /**
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @return mass of the body */
  double getMass(size_t bodyIdx) const final { return mass[bodyIdx]; }

  /**
   * set body mass. It is indexed for each body, not for individual link. Check this link
   * (https://raisim.com/sections/ArticulatedSystem.html#introduction)
   * to understand the difference between a link and a body */
  void setMass(size_t bodyIdx, double value) { mass[bodyIdx] = value; }

  /**
   * @return the total mass of the system.*/
  double getTotalMass() const { return compositeMass[0]; }

  /**
   * set external forces or torques expressed in the world frame acting on the COM of the body.
   * The external force is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent force
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] force force applied to the body (at the center of mass) */
  void setExternalForce(size_t bodyIdx, const Vec<3> &force) final;

  /** set external force acting on the point specified
   * The external force is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent force
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] frameOfForce the frame in which the force is expressed. Options: Frame::WORLD_FRAME, Frame::PARENT_FRAME or Frame::BODY_FRAME
   * @param[in] force the applied force
   * @param[in] frameOfPos the frame in which the position vector is expressed. Options: Frame::WORLD_FRAME, Frame::PARENT_FRAME or Frame::BODY_FRAME
   * @param[in] pos position at which the force is applied*/
  void setExternalForce(size_t bodyIdx, Frame frameOfForce, const Vec<3> &force, Frame frameOfPos, const Vec<3> &pos);

  /** set external force (expressed in the world frame) acting on the point (expressed in the body frame) specified
   * The external force is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent force
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] force the applied force
   * @param[in] pos position at which the force is applied*/
  void setExternalForce(size_t bodyIdx, const Vec<3> &pos, const Vec<3> &force) final {
    setExternalForce(bodyIdx, Frame::WORLD_FRAME, force, Frame::BODY_FRAME, pos);
  }

  /** set external force (expressed in the world frame) acting on the point specified by the frame
   * The external force is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent force
   * @param[in] frame_name the name of the frame where you want to applied the force. The force is applied to the origin of the frame, on the body where the frame is attached to.
   * @param[in] force the applied force in the world frame*/
  void setExternalForce(const std::string &frame_name, const Vec<3> &force) {
    auto &frame = getFrameByName(frame_name);
    Vec<3> force_b; force_b = frame.orientation * force;
    setExternalForce(frame.parentId, Frame::BODY_FRAME, force_b, Frame::BODY_FRAME, frame.position);
  }

  /**
   * set external torque.
   * The external torque is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent torque
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] torque_in_world_frame the applied torque expressed in the world frame */
  void setExternalTorque(size_t bodyIdx, const Vec<3> &torque_in_world_frame) final;

  /**
   * set external torque.
   * The external torque is applied for a single time step only.
   * You have to apply the force for every time step if you want persistent torque
   * @param[in] bodyIdx the body index. it can be retrieved by getBodyIdx()
   * @param[in] torque_in_body_frame the applied torque expressed in the body frame */
  void setExternalTorqueInBodyFrame(size_t bodyIdx, const Vec<3> &torque_in_body_frame) {
    Vec<3> torque_in_world_frame;
    matvecmul(rot_WB[bodyIdx], torque_in_body_frame, torque_in_world_frame);
    setExternalTorque(bodyIdx, torque_in_world_frame);
  }

  /**
   * returns the contact point velocity. The contactId is the order in the vector from Object::getContacts()
   * @param[in] contactId index of the contact vector which can be obtained by getContacts()
   * @param[out] vel the contact point velocity */
  void getContactPointVel(size_t contactId, Vec<3> &vel) const final;

  /**
   * @param[in] mode control mode. Can be either ControlMode::FORCE_AND_TORQUE or ControlMode::PD_PLUS_FEEDFORWARD_TORQUE */
  void setControlMode(ControlMode::Type mode) { controlMode_ = mode; }

  /**
   * @return control mode. Can be either ControlMode::FORCE_AND_TORQUE or ControlMode::PD_PLUS_FEEDFORWARD_TORQUE */
  ControlMode::Type getControlMode() const { return controlMode_; }

  /**
   * set PD targets.
   * @param[in] posTarget position target
   * @param[in] velTarget velocity target */
  void setPdTarget(const Eigen::VectorXd &posTarget, const Eigen::VectorXd &velTarget) {
    RSFATAL_IF(size_t(posTarget.rows()) != gcDim,
               "position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(size_t(velTarget.rows()) != dof,
               "the velocity target should have the same dimension as the degrees of freedom")
    qref_ = posTarget;
    uref_ = velTarget;
  }

  /**
   * get PD targets.
   * @param[out] posTarget position target
   * @param[out] velTarget velocity target */
  void getPdTarget(Eigen::VectorXd &posTarget, Eigen::VectorXd &velTarget) {
    RSFATAL_IF(size_t(posTarget.rows()) != gcDim,
               "position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(size_t(velTarget.rows()) != dof,
               "the velocity target should have the same dimension as the degrees of freedom")
    posTarget = qref_.e();
    velTarget = uref_.e();
  }

  /**
   * set PD targets.
   * @param[in] posTarget position target (dimension == getGeneralizedCoordinateDim())
   * @param[in] velTarget velocity target (dimension == getDOF()) */
  void setPdTarget(const raisim::VecDyn &posTarget, const raisim::VecDyn &velTarget) {
    RSFATAL_IF(size_t(posTarget.rows()) != gcDim,
               "position target should have the same dimension as the generalized coordinate")
    RSFATAL_IF(size_t(velTarget.rows()) != dof,
               "the velocity target should have the same dimension as the degrees of freedom")
    qref_ = posTarget;
    uref_ = velTarget;
  }

  /**
   * set P targets.
   * @param[in] posTarget position target (dimension == getGeneralizedCoordinateDim())*/
  template<class T>
  void setPTarget(const T &posTarget) {
    setControlMode(ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    RSFATAL_IF(size_t(posTarget.rows()) != gcDim,
               "position target should have the same dimension as the generalized coordinate")
    qref_ = posTarget;
  }

  /**
   * set D targets.
   * @param[in] velTarget velocity target (dimension == getDOF()) */
  template<class T>
  void setDTarget(const T &velTarget) {
    RSFATAL_IF(size_t(velTarget.rows()) != dof,
               "the velocity target should have the same dimension as the degrees of freedom")
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
   * get PD gains.
   * @param[out] pgain position gain (dimension == getDOF())
   * @param[out] dgain velocity gain (dimension == getDOF())*/
  void getPdGains(Eigen::VectorXd &pgain, Eigen::VectorXd &dgain) {
    pgain = kp_.e();
    dgain = kd_.e();
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
    RSFATAL_IF(size_t(pgain.rows()) != dof, "p gains should have the same dimension as the degrees of freedom")
    kp_ = pgain;
    if (jointType[0] == Joint::FLOATING) {
      for (size_t i = 0; i < 6; i++)
        kp_[i] = 0;
    }
    dampedDiagonalTermUpdated_ = false;
  }

  /**
   * set D gains.
   * @param[in] dgain velocity gain (dimension == getDOF())*/
  template<class T>
  void setDGains(const T &dgain) {
    RSFATAL_IF(size_t(dgain.rows()) != dof, "d gains should have the same dimension as the degrees of freedom")
    kd_ = dgain;
    if (jointType[0] == Joint::FLOATING) {
      for (size_t i = 0; i < 6; i++)
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
  void computeSparseInverse(const MatDyn &M, MatDyn &Minv) noexcept;

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
  ArticulatedSystemOption getOptions() const { return options_; }

  /**
   * @return a vector of body names (following the joint order) */
  const std::vector<std::string> &getBodyNames() const { return bodyName; }

  /**
   * @return a vector of visualized bodies */
  std::vector<VisObject> &getVisOb() { return visObj; };
  const std::vector<VisObject> &getVisOb() const { return visObj; };

  /**
   * @return a vector of visualized collision bodies */
  std::vector<VisObject> &getVisColOb() { return visColObj; };
  const std::vector<VisObject> &getVisColOb() const { return visColObj; };

  /**
   * @param[in] visObjIdx visual object index. Following the order specified by the vector getVisOb()
   * @param[out] rot orientation
   * @param[out] pos position */
  void getVisObPose(size_t visObjIdx, Mat<3, 3> &rot, Vec<3> &pos) const { getPose(visObj[visObjIdx], rot, pos); }

  /**
   * @param[in] visColObjIdx visual collision object index. Following the order specified by the vector getVisColOb()
   * @param[out] rot orientation
   * @param[out] pos position */
  void getVisColObPose(size_t visColObjIdx, Mat<3, 3> &rot, Vec<3> &pos) const {
    getPose(visColObj[visColObjIdx],
            rot,
            pos);
  }

  /**
   * @return the resource directory (for mesh files, textures, etc) */
  const std::string &getResourceDir() const { return resourceDir_; }

  /**
   * @return the resource directory (for mesh files, textures, etc) */
  const std::string &getRobotDescriptionfFileName() const { return robotDefFileName_; }

  /**
   * @return the name of the URDF file (returns empty string if the robot was not specified by a URDF file) */
  const std::string &getRobotDescriptionfTopDirName() const { return robotDefFileUpperDir_; }

  /**
   * @return the full path to the URDF file (returns empty string if the robot was not specified by a URDF file) */
  const std::string &getRobotDescriptionFullPath() const { return fullURDFPath_; }

  /**
   * @return if the object was instantiated with raw URDF string, it returns the string */
  const std::string &getRobotDescription() const { return robotDef_; }

  /**
   * if the object was instantiated with raw URDF string, it exports the robot description to an URDF file
   * @param[in] filePath Path where the file is generated */
  void exportRobotDescriptionToURDF(const std::string &filePath) const {
    RSFATAL_IF(robotDef_.empty(), "This articulated system was created with an existing urdf file")
    std::ofstream urdfFile;
    RSFATAL_IF(filePath.substr(filePath.size() - 5, 5) != ".urdf", "Articulated System is only saved to a URDF format")
    urdfFile.open(filePath);
    urdfFile << robotDef_;
    urdfFile.close();
  }

  /**
   * set the base position using an eigen vector
   * @param[in] pos position of the base */
  void setBasePos_e(const Eigen::Vector3d &pos) {
    if (jointType[0] == Joint::FIXED)
      jointPos_W[0].e() = pos;
    else {
      gc_[0] = pos[0];
      gc_[1] = pos[1];
      gc_[2] = pos[2];
    }
    updateKinematics();
  }

  /**
   * set the base orientation using an eigen vector
   * @param[in] rot orientation of the base */
  void setBaseOrientation_e(const Eigen::Matrix3d &rot) {
    if (jointType[0] == Joint::FIXED)
      rot_WB[0].e() = rot;
    else {
      Mat<3, 3> rotMat{};
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
  void setBasePos(const Vec<3> &pos);
  /**
   * set the base orientation using an eigen vector
   * @param[in] rot orientation of the base */
  void setBaseOrientation(const Mat<3, 3> &rot);

  /**
   * set limits in actuation force. It can be also specified in the URDF file
   * @param[in] upper upper joint force/torque limit
   * @param[in] lower lower joint force/torque limit*/
  void setActuationLimits(const Eigen::VectorXd &upper, const Eigen::VectorXd &lower) {
    tauUpper_ = upper;
    tauLower_ = lower;
  }

  /**
   * @return the upper joint torque/force limit*/
  const VecDyn &getActuationUpperLimits() const { return tauUpper_; }

  /**
   * @return the lower joint torque/force limit*/
  const VecDyn &getActuationLowerLimits() const { return tauLower_; }

  /**
   * change collision geom parameters.
   * @param[in] id collision object id
   * @param[in] params collision object parameters (depending on the object).
   * For a sphere, {raidus}.
   * For a cylinder and a capsule, {radius, length}
   * For a box, {x-dim, y-dim, z-dim} */
  void setCollisionObjectShapeParameters(size_t id, const std::vector<double> &params);

  /**
   * change collision geom offset from the joint position.
   * @param[in] id collision object id
   * @param[in] posOffset the position vector expressed in the joint frame */
  void setCollisionObjectPositionOffset(size_t id, const Vec<3> &posOffset);

  /**
   * change collision geom orientation offset from the joint frame.
   * @param[in] id collision object id
   * @param[in] oriOffset the orientation relative to the joint frame */
  void setCollisionObjectOrientationOffset(size_t id, const Mat<3, 3> &oriOffset);

  /**
   * rotor inertia is a term added to the diagonal of the mass matrix. This approximates the rotor inertia. Note that this
   * is not exactly equivalent in dynamics (due to gyroscopic effect). but it is a commonly used approximation.
   * It can also be expressed in the URDF file
   * @param[in] rotorInertia the rotor inertia */
  void setRotorInertia(const VecDyn &rotorInertia) {
    rotorInertia_ = rotorInertia;
  }

  /**
   * rotor inertia is a term added to the diagonal of the mass matrix. This approximates the rotor inertia. Note that this
   * is not exactly equivalent in dynamics (due to gyroscopic effect). but it is a commonly used approximation.
   * It can also be expressed in the URDF file
   * @return the rotor inertia */
  const VecDyn &getRotorInertia() const {
    return rotorInertia_;
  }

  /**
   * This joint indices are in the same order as the elements of the generalized velocity
   * However, some joints have multiple degrees of freedom and they are not equal
   * @param[in] jointIndex the joint index
   * @return the joint type */
  Joint::Type getJointType(size_t jointIndex) { return jointType[jointIndex]; }
  const Joint::Type getJointType(size_t jointIndex) const { return jointType[jointIndex]; }

  /**
   * @return the number of joints (same as the number of bodies) */
  size_t getNumberOfJoints() const { return nbody; }

  /**
   * returns reference object of the joint
   * @param[in] name joint name
   * @return a JointRef to the joint. Check the example JointRefAndLinkRef*/
  JointRef getJoint(const std::string &name) {
    auto index = getFrameIdxByName(name);
    return {index, this};
  }

  /**
   * returns reference object of the joint
   * @param[in] name the link name
   * @return a LinkRef to the link. Check the example JointRefAndLinkRef*/
  LinkRef getLink(const std::string &name) {
    return {size_t(std::find(bodyName.begin(), bodyName.end(), name) - bodyName.begin()), this};
  }

  /**
   * @return a mapping that converts body index to gv index */
  const std::vector<size_t> &getMappingFromBodyIndexToGeneralizedVelocityIndex() const {
    return bodyIdx2GvIdx;
  }

  /**
   * @return a mapping that converts body index to gv index */
  const std::vector<size_t> &getMappingFromBodyIndexToGeneralizedCoordinateIndex() const {
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
  std::vector<contact::Single3DContactProblem const *> getJointLimitViolations(const contact::ContactProblems & problemListFromWorld) {
    std::vector<contact::Single3DContactProblem const *> vec;
    for(auto v: jointLimitViolation_)
      vec.push_back(&problemListFromWorld[v]);
    return vec;
  }

  /**
   * set new joint limits
   * For revolute and prisimatic joints, the joint limit is {lower, upper}
   * For spherical joint, the joint limit is {angle, NOT_USED}
   * @param[in] jointLimits joint limits*/
  void setJointLimits(const std::vector<raisim::Vec<2>> &jointLimits) {
    jointLimits_ = jointLimits;
  }

  /**
   * get the joint limits
   * For revolute and prisimatic joints, the joint limit is {lower, upper}
   * For spherical joint, the joint limit is {angle, NOT_USED}
   * @return jointLimits joint limits*/
  const std::vector<raisim::Vec<2>> &getJointLimits() {
    return jointLimits_;
  }

  /**
   * Clears all external forces and torques */
  void clearExternalForcesAndTorques() {
    isExternalForces_.resize(0);
    externalForceAndTorque_.resize(0);
    externalForceAndTorquePos_.resize(0);
    externalForceViz_.resize(0);
    externalForceVizPos_.resize(0);
    externalTorqueViz_.resize(0);
    externalTorqueVizPos_.resize(0);
    externalForceAndTorqueJaco_.resize(0);
  }

  /**
   * @param[in] spring Additional spring elements for joints */
  void addSpring(const SpringElement &spring) { springs_.push_back(spring); }

  /**
   * @return springs Existing spring elements on joints */
  std::vector<SpringElement> &getSprings() { return springs_; }
  const std::vector<SpringElement> &getSprings() const { return springs_; }

  /**
   *
   * @return parent parent[i] is a parent body id of the i^th body
   */
  const std::vector<size_t>& getParentVector() const { return parent; }

  // not recommended for users. only for developers
  void addConstraints(const std::vector<PinConstraintDefinition>& pinDef);
  void initializeConstraints();

 protected:

  ArticulatedSystem(const std::string &filePath,
                    const std::string &xmlScript,
                    const std::string &resDir,
                    const std::vector<std::string> &jointOrder,
                    ArticulatedSystemOption options);

  void getPose(const VisObject &vob, Mat<3, 3> &rot, Vec<3> &pos) const;

  void getSparseJacobian_internal(size_t bodyIdx, const Vec<3> &point_W, SparseJacobian &jaco);

  void updateCollision() final;

  void computeMassMatrix(MatDyn &M) noexcept;

  void computeNonlinearities(const Vec<3> &gravity, VecDyn &b);

  void destroyCollisionBodies(dSpaceID id) final;

  /* This computes Delassus matrix necessary for contact force computation */
  void preContactSolverUpdate1(const Vec<3> &gravity, double dt) final;
  void preContactSolverUpdate2(const Vec<3> &gravity, double dt, contact::ContactProblems& problems) final;
  void integrateWithoutContact(const Vec<3> &gravity, double dt);

  void integrate(double dt, const Vec<3> &gravity) final;

  void addContactPointVel(size_t pointId, Vec<3> &vel) final;

  void subContactPointVel(size_t pointId, Vec<3> &vel) final;

  void updateGenVelWithImpulse(size_t pointId, const Vec<3> &imp) final;

  void updateTimeStep(double dt) final;

  void updateTimeStepIfNecessary(double dt) final;

  inline void jacoSub(const raisim::SparseJacobian &jaco1, raisim::SparseJacobian &jaco, bool isFloatingBase);

  void setConstraintForce(size_t bodyIdx, const Vec<3> &pos, const Vec<3> &force) final;

 private:

  void init();

  void integratePosition(VecDyn &gc, VecDyn &gvAvg, double dt);

  void computeExpandedParentArray();

  double enforceJointLimits(contact::Single3DContactProblem &problem) final;

  void computeDampedMass(double dt);

  void cleanContacts() {
    contactJaco_.clear();
    getContacts().clear();
  }

  void rkIntegrate(const Vec<3> &gravity, double dt);

  void clearExternalForces();

  void appendConstraints(contact::ContactProblems& problems) final;

  /// to be removed. just for testing purposes
 public:
  void articulatedBodyAlgorithm(const Vec<3> &gravity, double dt);
  const std::vector<MatDyn>& getMinvJT() { return MinvJT_T; }
  const std::vector<VecDyn>& getj_MinvJT_T1D() { return j_MinvJT_T1D; }
  const raisim::VecDyn& getUdot() { return udot_; }
  void getFullDelassusAndTauStar(double dt);
  void appendJointLimits(contact::ContactProblems &problem) final;

 private:
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
  Mat<3,3> fixedBaseOri_;

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
  VecDyn gc_, gcOld_, gv_, udot_, gvOld_, gvAvg_, h_, gvERP_, gvTemp_, gvPreimpact_, gcRK_[4], gvRK_[4], slopeRK_[4];
  MatDyn M_, Minv_, lT_;

  /// Contact variables
  std::vector<MatDyn> MinvJT_T;
  std::vector<VecDyn> j_MinvJT_T1D;
  VecDyn tauStar_, tau_, tauFF_;
  VecDyn tauUpper_, tauLower_, velLimits_; // bounds
  std::vector<size_t> bodyIdx2GvIdx, bodyIdx2GcIdx;

  std::vector<SparseJacobian> J_;
  std::vector<CoordinateFrame> frameOfInterest_;

 protected:
  std::vector<Child> rootChild_;
  std::vector<std::string> bodyName;
  std::vector<std::string> movableJointNames;

  std::vector<SparseJacobian> contactJaco_;
  std::vector<SparseJacobian> externalForceAndTorqueJaco_;

  raisim::CollisionSet collisionBodies;
  std::vector<VisObject> visColObj, visObj;
  ArticulatedSystemOption options_;

 private:
  size_t nbody, dof = 0, gcDim = 0;
  bool dampedDiagonalTermUpdated_ = false;
  ControlMode::Type controlMode_ = ControlMode::FORCE_AND_TORQUE;
  raisim::SparseJacobian tempJaco_;
  double dt_ = 0.001;

  // damping
  VecDyn cd_;
  VecDyn cf_;

  std::vector<SpringElement> springs_;

  // stable PD controller
  VecDyn kp_, kd_, uref_, qref_, diag_w, diag_w_dt_, posErr_, uErr_;
  VecDyn rotorInertia_;
  VecDyn generalizedMomentum_;
  VecDyn temp, temp2;
  MatDyn Mtemp_;
  std::string resourceDir_, robotDefFileName_, robotDefFileUpperDir_, fullURDFPath_, robotDef_;

  // for Trimesh
  std::vector<dTriMeshDataID> meshData_;
  std::vector<std::vector<float>> meshVertices_;
  std::vector<std::vector<dTriIndex>> meshIdx_;

  // integration scheme
  IntegrationScheme desiredIntegrationScheme_ = IntegrationScheme::TRAPEZOID;
  IntegrationScheme integrationSchemeThisTime_;

  // joint violations
  std::vector<size_t> jointLimitViolation_;

  // constraints
  std::vector<PinConstraint> pinConstraints_;
  std::vector<PinConstraintDefinition> pinDef_;

  /// ABA
  Mat<6, 6> MaInv_base;
  std::vector<Eigen::Matrix<double, 3, 3>, AlignedAllocator<Eigen::Matrix<double, 3, 3>, 32>> joint2Com_w_Skew;

  struct AbaData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 6, 6> XT;
    Eigen::Matrix<double, 3, 6>  XcT;
    Eigen::Matrix<double, 1, 6>  XcT1D;
    Eigen::Matrix<double, 1, 6> STMaSinvSTMaXT;
    Eigen::Matrix<double, 1, 6> ST;
    Eigen::Matrix<double, 6, 1> acc;
    Eigen::Matrix<double, 6, 1> SdotUpXdotTV;
    double STMaSinv;
    double udotExpectAccTerm;
    Eigen::Matrix<double, 6, 6> XMXT;
    Eigen::Matrix<double, 1, 6> STMaXT;
    Eigen::Matrix<double, 1, 6> SdotT;
    Eigen::Matrix<double, 1, 6> STMa;

    Eigen::Matrix<double, 6, 6> Ma;
    Eigen::Matrix<double, 6, 1> Pa;
  };

  struct AbaData3 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 3, 6> STMaXT3, ST3, SdotT3, STMa3, STMaSinvSTMaXT3;
    Eigen::Matrix<double, 3, 3> STMaSinv3, STMaS3;
    Eigen::Matrix<double, 3, 1> udotExpectAccTerm3;
  };

  std::vector<AbaData, AlignedAllocator<AbaData, 32>> ad_;
  std::vector<AbaData3, AlignedAllocator<AbaData3, 32>> ad3_;
};
}

#endif //RAISIM_ARTICULATEDSYSTEM_HPP
