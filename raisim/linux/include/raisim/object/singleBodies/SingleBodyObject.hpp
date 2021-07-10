//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_SINGLEBODYOBJECT_HPP
#define RAISIM_SINGLEBODYOBJECT_HPP

#include "Eigen/Geometry"
#include "raisim/object/Object.hpp"
#include "raisim/math.hpp"

namespace raisim {

enum GyroscopicMode {
  IMPLICIT_GYROSCOPIC_FORCE_BODY,     // implicit body model (stable, more computation)
  IMPLICIT_GYROSCOPIC_FORCE_WORLD,    // implicit world model (stable, more computation)
  EXPLICIT_GYROSCOPIC_FORCE,          // explicit model (unstable, less computation)
  NO_GYROSCOPIC_FORCE
};

/// this class is only for inheritance
class SingleBodyObject : public Object {

 public:
  explicit SingleBodyObject(ObjectType singleBodyObjectType);

  /**
   * returns the quaternion in Eigen::Vector4d
   * @return the orientation of the object */
  inline Eigen::Vector4d getQuaternion() const {
    Eigen::Vector4d quat = bodyQuaternion_.e();
    return quat;
  };

  /**
   * returns the quaternion in raisim::Vec<4>
   * @param[out] quat the orientation of the object */
  inline void getQuaternion(Vec<4>& quat) const {
    quat = bodyQuaternion_;
  }

  /**
   * returns the rotation matrix in Eigen::Matrix3d
   * @return the orientation of the object */
  inline Eigen::Matrix3d getRotationMatrix() const {
    Eigen::Matrix3d rot = bodyRotationMatrix_.e();
    return rot;
  };

  /**
   * returns the quaternion in raisim::Mat<3,3>
   * @param[out] rotation the orientation of the object */
  inline void getRotationMatrix(Mat<3,3>& rotation) const {
    rotation = bodyRotationMatrix_;
  }


  /**
   * returns the body position in Eigen::Vector3d.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  inline Eigen::Vector3d getPosition() const {
    Eigen::Vector3d pos = bodyPosition_.e();
    return pos;
  };

  /**
   * returns the body position in Eigen::Vector3d.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  inline Eigen::Vector3d getComPosition() const {
    Eigen::Vector3d pos = comPosition_.e();
    return pos;
  };

  /**
   * returns the body position in raisim::Vec<3>.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  inline const raisim::Vec<3>& getComPosition_rs() const {
    return comPosition_;
  };

  /**
   * returns the body position in raisim::Vec<3>.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  inline const raisim::Vec<3>& getBodyToComPosition_rs() const {
    return body2com_;
  };

  /**
   * returns the linear velocity
   * @return the linear velocity of the object */
  inline Eigen::Vector3d getLinearVelocity() const {
    Eigen::Vector3d vel = linVel_.e();
    return vel;
  };

  /**
   * returns the linear velocity
   * @param[out] linVel the linear velocity of the object */
  inline void getLinearVelocity(Vec<3>& linVel) { linVel = linVel_; }

  /**
   * returns the angular velocity
   * @return the angular velocity of the object */
  inline Eigen::Vector3d getAngularVelocity() const {
    Eigen::Vector3d vel = angVel_.e();
    return vel;
  };

  /**
   * returns the angular velocity
   * @param[out] angVel the angular velocity of the object */
  inline void getAngularVelocity(Vec<3>& angVel) { angVel = angVel_; }

  inline void getPosition(size_t localIdx, Vec<3>& pos_w) const final {
    pos_w = bodyPosition_;
  }

  inline void getOrientation(size_t localIdx, Mat<3,3>& rot) const final {
    rot = bodyRotationMatrix_;
  }

  /**
   * returns the kinetic energy
   * @return the kinetic energy of the object */
  double getKineticEnergy() const;

  /**
   * returns the potential energy w.r.t. z=0 and the given gravitational acceleration
   * param[in] gravity gravitational acceleration
   * @return the potential energy of the object */
  double getPotentialEnergy(const Vec<3> &gravity) const;

  /**
   * equivalent to getKineticEnergy() + getPotentialEnergy(gravity)
   * param[in] gravity gravitational acceleration
   * @return the sum of the potential and gravitational energy of the object */
  double getEnergy(const Vec<3> &gravity) const;

  /**
   * returns the linear momentum of the object
   * @return the linear momentum of the object */
  Eigen::Vector3d getLinearMomentum() const;

  /**
   * returns the mass of the object. The localIdx is unused
   * @return the linear momentum of the object */
  double getMass(size_t localIdx = 0) const override;

  /**
   * set the mass of the object.
   * @param[in] mass set the mass of the object */
  inline void setMass(double mass) {
    mass_ = mass; inverseMass_ = 1./mass;
  }

  /**
   * get the inertia matrix in the body frame.
   * This value is constant.
   * @return the inertia matrix in the body frame */
  inline Eigen::Matrix3d getInertiaMatrix_B() const {
    Eigen::Matrix3d iner = inertia_b_.e();
    return iner;
  }

  /**
   * get the inertia matrix in the world frame.
   * This value changes as the body rotates.
   * @return the inertia matrix in the world frame */
  inline Eigen::Matrix3d getInertiaMatrix_W() const {
    Eigen::Matrix3d iner = inertia_w_.e();
    return iner;
  }

  /**
   * get the inertia matrix in the body frame (raisim matrix type).
   * This value is constant.
   * @return the inertia matrix in the body frame */
  const raisim::Mat<3,3>& getInertiaMatrix_B_rs() const {
    return inertia_b_;
  }

  /**
   * get the inertia matrix in the world frame (raisim matrix type).
   * This value changes as the body rotates.
   * @return the inertia matrix in the world frame */
  const raisim::Mat<3,3>& getInertiaMatrix_W_rs() const {
    return inertia_w_;
  }

  ObjectType getObjectType() const final;
  dGeomID getCollisionObject() const;

  GyroscopicMode getGyroscopicMode() const;
  virtual void setPosition(const Eigen::Vector3d &originPosition);
  virtual void setPosition(double x, double y, double z);
  virtual void setPosition(const Vec<3>& pos);
  virtual void setOrientation(const Eigen::Quaterniond &quaternion) {
    bodyQuaternion_ = {quaternion.w(),
                       quaternion.x(),
                       quaternion.y(),
                       quaternion.z()};
    bodyQuaternion_ /= bodyQuaternion_.norm();
    updateOrientation();
  }
  virtual void setOrientation(const Eigen::Vector4d &quaternion) {
    bodyQuaternion_.e() = quaternion;
    updateOrientation();
  }
  virtual void setOrientation(double w, double x, double y, double z) {
    bodyQuaternion_ = {w, x, y, z};
    updateOrientation();
  }
  virtual void setOrientation(const Eigen::Matrix3d &rotationMatrix) {
    Eigen::Quaterniond quaternion(rotationMatrix);
    setOrientation(quaternion);
  }
  virtual void setOrientation(const Vec<4>& quat) {
    bodyQuaternion_ = quat;
    updateOrientation();
  }

  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Quaterniond &quaternion);
  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Vector4d &quaternion);
  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Matrix3d &rotationMatrix);
  void setInertia(const Eigen::Matrix3d& inertia);
  void setInertia(const Mat<3,3>& inertia) {
    inertia_b_ = inertia;
    cholInv<3>(inertia_b_.ptr(), inverseInertia_b_.ptr());
    updateWorldInertia();
  }

  const Vec<3>& getCom() { return body2com_; }
  void setCom(const Vec<3>& com) { body2com_ = com; }

  virtual void setVelocity(const Eigen::Vector3d &linearVelocity, const Eigen::Vector3d &angularVelocity) {
    linVel_.e() = linearVelocity;
    angVel_.e() = angularVelocity;
  }

  inline void setVelocity(const Vec<3>& linVel, const Vec<3>& angVel) { linVel_ = linVel; angVel_ = angVel; }
  
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
    linVel_ = {dx, dy, dz};
    angVel_ = {wx, wy, wz};
  }

  void setLinearVelocity(const Vec<3>& linVel) { linVel_ = linVel; }

  void setAngularVelocity(const Vec<3>& angVel) { angVel_ = angVel; }

  void setExternalForce(size_t localIdx, const Vec<3>& force) final;
  void setExternalTorque(size_t localIdx, const Vec<3>& torque) final;
  void setExternalForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) final;
  void setConstraintForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) final;
  void setGyroscopicMode(GyroscopicMode gyroscopicMode);
  void getPosition(size_t localIdx, const Vec<3>& pos_b, Vec<3>& pos_w) const final;
  void getPosition(Vec<3>& pos_w) { pos_w = bodyPosition_; };
  void getVelocity(size_t localIdx, Vec<3>& vel_w) const final { vel_w = linVel_; }

  void preContactSolverUpdate1(const Vec<3> &gravity, double dt) final;
  void preContactSolverUpdate2(const Vec<3> &gravity, double dt, contact::ContactProblems& problems) final;
  void integrate(double dt, const Vec<3>& gravity) final;
  void getContactPointVel(size_t pointId, Vec<3> &vel) const final;

  virtual void updateCollision() override;

  void setLinearDamping(double damping);
  void setAngularDamping(Vec<3> damping);
  virtual void setBodyType(BodyType type);

  CollisionGroup getCollisionGroup();
  CollisionGroup getCollisionMask();

  void setAppearance(const std::string& appearance) { appearance_ = appearance; }
  const std::string& getAppearance() const { return appearance_; }

  void clearExternalForcesAndTorques() override {
    isExternalForces_.resize(0);
    externalForceAndTorque_.resize(0);
    externalForceAndTorquePos_.resize(0);
    externalForceViz_.resize(0);
    externalForceVizPos_.resize(0);
    externalTorqueViz_.resize(0);
    externalTorqueVizPos_.resize(0);
  }

 protected:

  virtual void destroyCollisionBodies(dSpaceID id) override;
  void updateWorldInertia();
  void addContactPointVel(size_t pointId, Vec<3>& vel) final;
  void subContactPointVel(size_t pointId, Vec<3>& vel) final;
  void updateGenVelWithImpulse(size_t pointId, const Vec<3>& imp) final;
  void updateTimeStep(double dt) final {};
  void updateTimeStepIfNecessary(double dt) final {};
  void updateOrientation();


  dGeomID collisionObject_;

  // object type
  ObjectType singleBodyObjectType_;

  /// NOTE
  /// each single body object has 3 frame
  /// 1. body frame (coincides with graphical object)
  /// 2. com frame
  /// 3. bullet frame

  // body pose
  /// body frame does NOT coincides with com frame
  Vec<4> bodyQuaternion_;
  Mat<3, 3> bodyRotationMatrix_;
  Vec<3> bodyPosition_;

  // center of mass pose
  /// com frame orientation always coincides with world frame
  Vec<3> comPosition_;
  Vec<3> body2com_;     // w.r.t body frame

  // bullet pose
  /// bullet frame orientation always coincides with body frame
  Vec<3> colPosition_;
  Vec<3> col2com_;     // w.r.t bullet frame

  // object velocity
  /// com velocity and angular velocity w.r.t com
  Vec<3> linVel_, oldLinVel_;
  Vec<3> angVel_;
  Vec<6> genVel_;

  // physical properties
  Mat<3, 3> inertia_b_;
  Mat<3, 3> inertia_w_;
  Mat<3, 3> inverseInertia_b_;
  Mat<3, 3> inverseInertia_w_;

  // forces
  GyroscopicMode gyroscopicMode_ = IMPLICIT_GYROSCOPIC_FORCE_BODY;
  Vec<3> h_f_, h_tau_;
  Vec<3> taustar_f_, taustar_tau_;
  Vec<3> tau_f_, tau_tau;    // external forces

  // -r (vector from contact point to c.o.m)
  std::vector<Vec<3>> contact2com_;

  double mass_, inverseMass_;
  double linearDamping_=0;
  Vec<3> angularDamping_;

  std::vector <raisim::Mat<6,3>> MinvJT_;
  std::vector <raisim::Mat<3,6>> J_;

  /// constrained systems use different integration scheme
  bool constrained_ = false;

  /// for visualization
  std::string appearance_;
};

} // rai_simulator

#endif //RAISIM_SINGLEBODYOBJECT_HPP
