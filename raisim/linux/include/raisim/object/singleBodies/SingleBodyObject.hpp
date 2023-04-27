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
  [[nodiscard]] inline Eigen::Vector4d getQuaternion() const {
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
  [[nodiscard]] inline Eigen::Matrix3d getRotationMatrix() const {
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
  [[nodiscard]] inline Eigen::Vector3d getPosition() const {
    Eigen::Vector3d pos = bodyPosition_.e();
    return pos;
  };

  /**
   * returns the body position in Eigen::Vector3d.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  [[nodiscard]] inline Eigen::Vector3d getComPosition() const {
    Eigen::Vector3d pos = comPosition_.e();
    return pos;
  };

  /**
   * returns the body position in raisim::Vec<3>.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  [[nodiscard]] inline const raisim::Vec<3>& getComPosition_rs() const {
    return comPosition_;
  };

  /**
   * returns the body position in raisim::Vec<3>.
   * Currently, all body positions are the same as the COM position
   * @return the position of the object */
  [[nodiscard]] inline const raisim::Vec<3>& getBodyToComPosition_rs() const {
    return body2com_;
  };

  /**
   * returns the linear velocity
   * @return the linear velocity of the object */
  [[nodiscard]] inline Eigen::Vector3d getLinearVelocity() const {
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
  [[nodiscard]] inline Eigen::Vector3d getAngularVelocity() const {
    Eigen::Vector3d vel = angVel_.e();
    return vel;
  };

  /**
   * returns the angular velocity
   * @param[out] angVel the angular velocity of the object */
  inline void getAngularVelocity(Vec<3>& angVel) { angVel = angVel_; }

  /**
   * returns position vector.
   * @param[in] localIdx this should be always 0 (this method is just to keep the same api as the ArticulatedSystem class)
   * @param[out] pos_w the position vector of the object */
  inline void getPosition(size_t localIdx, Vec<3>& pos_w) const final {
    pos_w = bodyPosition_;
  }


  /**
   * returns the orientation of the object
   * @param[in] localIdx local idx should be always 0 (this method is just to keep the same api as the ArticulatedSystem class)
   * @param[out] pos_w the rotation matrix of the object */
  inline void getOrientation(size_t localIdx, Mat<3,3>& rot) const final {
    rot = bodyRotationMatrix_;
  }

  /**
   * returns the rotation matrix
   * @return rotation matrix */
    inline const Mat<3,3>& getOrientation() const {
      return bodyRotationMatrix_;
    }

  /**
   * returns the kinetic energy
   * @return the kinetic energy of the object */
  [[nodiscard]] double getKineticEnergy() const;

  /**
   * returns the potential energy w.r.t. z=0 and the given gravitational acceleration
   * @param[in] gravity gravitational acceleration
   * @return the potential energy of the object */
  [[nodiscard]] double getPotentialEnergy(const Vec<3> &gravity) const;

  /**
   * equivalent to getKineticEnergy() + getPotentialEnergy(gravity)
   * @param[in] gravity gravitational acceleration
   * @return the sum of the potential and gravitational energy of the object */
  [[nodiscard]] double getEnergy(const Vec<3> &gravity) const;

  /**
   * returns the linear momentum of the object
   * @return the linear momentum of the object */
  [[nodiscard]] Eigen::Vector3d getLinearMomentum() const;

  /**
   * returns the mass of the object. The localIdx is unused
   * @return the linear momentum of the object */
  [[nodiscard]] double getMass(size_t localIdx = 0) const override;

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
  [[nodiscard]] inline Eigen::Matrix3d getInertiaMatrix_B() const {
    Eigen::Matrix3d iner = inertia_b_.e();
    return iner;
  }

  /**
   * get the inertia matrix in the world frame.
   * This value changes as the body rotates.
   * @return the inertia matrix in the world frame */
  [[nodiscard]] inline Eigen::Matrix3d getInertiaMatrix_W() const {
    Eigen::Matrix3d iner = inertia_w_.e();
    return iner;
  }

  /**
   * get the inertia matrix in the body frame (raisim matrix type).
   * This value is constant.
   * @return the inertia matrix in the body frame */
  [[nodiscard]] const raisim::Mat<3,3>& getInertiaMatrix_B_rs() const {
    return inertia_b_;
  }

  /**
   * get the inertia matrix in the world frame (raisim matrix type).
   * This value changes as the body rotates.
   * @return the inertia matrix in the world frame */
  [[nodiscard]] const raisim::Mat<3,3>& getInertiaMatrix_W_rs() const {
    return inertia_w_;
  }

  [[nodiscard]] ObjectType getObjectType() const final;
  [[nodiscard]] dGeomID getCollisionObject() const;

  [[nodiscard]] GyroscopicMode getGyroscopicMode() const;

  /**
   * Set position of the object (using Eigen)
   * @param[in] originPosition Position
   */
  virtual void setPosition(const Eigen::Vector3d &originPosition);

  /**
   * Set position of the object (using three doubles)
   * @param[in] x x position
   * @param[in] y y position
   * @param[in] z z position
   */
  virtual void setPosition(double x, double y, double z);

  /**
   * Set position of the object (using raisim::Vec<3>)
   * @param[in] originPosition Position
   */
  virtual void setPosition(const Vec<3>& pos);

  /**
   * Set orientation of the object (using Eigen::Quaterniond)
   * @param[in] quaternion quaternion
   */
  virtual void setOrientation(const Eigen::Quaterniond &quaternion) {
    bodyQuaternion_ = {quaternion.w(),
                       quaternion.x(),
                       quaternion.y(),
                       quaternion.z()};
    bodyQuaternion_ /= bodyQuaternion_.norm();
    updateOrientation();
  }

  /**
   * Set orientation of the object (using Eigen::Vector4d)
   * @param[in] quaternion quaternion
   */
  virtual void setOrientation(const Eigen::Vector4d &quaternion) {
    bodyQuaternion_.e() = quaternion;
    updateOrientation();
  }

  /**
   * Set orientation of the object (using doubles)
   * @param[in] w w
   * @param[in] x w
   * @param[in] y w
   * @param[in] z w
   */
  virtual void setOrientation(double w, double x, double y, double z) {
    bodyQuaternion_ = {w, x, y, z};
    updateOrientation();
  }

  /**
   * Set orientation of the object (using Eigen::Matrix3d)
   * @param[in] rotationMatrix rotation matrix
   */
  virtual void setOrientation(const Eigen::Matrix3d &rotationMatrix) {
    Eigen::Quaterniond quaternion(rotationMatrix);
    setOrientation(quaternion);
  }

  /**
   * Set orientation of the object (using Vec<4>)
   * @param[in] quat quaternion
   */
  virtual void setOrientation(const Vec<4>& quat) {
    bodyQuaternion_ = quat;
    updateOrientation();
  }

  /**
   * Set both the position and orientation of the object (using Eigen::Vector3d and Eigen::Quaterniond)
   * @param[in] originPosition position
   * @param[in] quaternion quaternion
   */
  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Quaterniond &quaternion);

  /**
   * Set both the position and orientation of the object (using Eigen::Vector3d and Eigen::Vector4d)
   * @param[in] originPosition position
   * @param[in] quaternion quaternion
   */
  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Vector4d &quaternion);

  /**
   * Set both the position and orientation of the object (using Eigen::Vector3d and Eigen::Matrix3d)
   * @param[in] originPosition position
   * @param[in] rotationMatrix rotation matrix
   */
  virtual void setPose(const Eigen::Vector3d &originPosition, const Eigen::Matrix3d &rotationMatrix);


  /**
   * Set inertia of the object (using Eigen::Matrix3d)
   * @param[in] inertia inertia of the object
   */
  void setInertia(const Eigen::Matrix3d& inertia);

  /**
   * Set inertia of the object (using raisim::Mat<3,3>)
   * @param[in] inertia inertia of the object
   */
  void setInertia(const Mat<3,3>& inertia) {
    inertia_b_ = inertia;
    cholInv<3>(inertia_b_.ptr(), inverseInertia_b_.ptr());
    updateWorldInertia();
  }

  /**
   * get the center of mass position
   * @return the position of the center of mass
   */
  const Vec<3>& getCom() { return body2com_; }

  /**
   * set the center of mass position
   * @param[in] com the position of the center of mass
   */
  void setCom(const Vec<3>& com) { body2com_ = com; }

  /**
   * set both the linear and angular velocity of the object (using Eigen::Vector3d)
   * @param[in] linearVelocity the linear velocity of the object
   * @param[in] angularVelocity  the angular velocity of the object
   */
  virtual void setVelocity(const Eigen::Vector3d &linearVelocity, const Eigen::Vector3d &angularVelocity) {
    linVel_.e() = linearVelocity;
    angVel_.e() = angularVelocity;
  }

  /**
   * set both the linear and angular velocity of the object (using raisim::Vec<3>)
   * @param[in] linearVelocity the linear velocity of the object
   * @param[in] angularVelocity  the angular velocity of the object
   */
  inline void setVelocity(const Vec<3>& linearVelocity, const Vec<3>& angularVelocity) { linVel_ = linearVelocity; angVel_ = angularVelocity; }

  /**
   * set both the linear and angular velocity of the object (using 6 doubles)
   * @param[in] dx the x-axis linear velocity of the object
   * @param[in] dy the y-axis linear velocity of the object
   * @param[in] dz the z-axis linear velocity of the object
   * @param[in] wx the x-axis angular velocity of the object
   * @param[in] wy the y-axis angular velocity of the object
   * @param[in] wz the z-axis angular velocity of the object
   */
  virtual void setVelocity(double dx, double dy, double dz, double wx, double wy, double wz) {
    linVel_ = {dx, dy, dz};
    angVel_ = {wx, wy, wz};
  }

  /**
   * set only the linear velocity
   * @param[in] linearVelocity the linear velocity
   */
  void setLinearVelocity(const Vec<3>& linearVelocity) { linVel_ = linearVelocity; }

  /**
   * set only the angular velocity
   * @param[in] angVel the angular velocity
   */
  void setAngularVelocity(const Vec<3>& angularVelocity) { angVel_ = angularVelocity; }

  /**
   * set external force on the object
   * @param[in] localIdx this should always be 0 (because the single body object only has one body)
   * @param[in] force force acting on the center of the mass (expressed in the world frame)
   */
  void setExternalForce(size_t localIdx, const Vec<3>& force) final;

  /**
   * set external torque on the object
   * @param[in] localIdx this should always be 0 (because the single body object only has one body)
   * @param[in] torque torque acting on body (expressed in the world frame)
   */
  void setExternalTorque(size_t localIdx, const Vec<3>& torque) final;

  /**
   * set external force on the object
   * @param[in] localIdx this should always be 0 (because the single body object only has one body)
   * @param[in] pos the application point of the force (expressed in the world frame)
   * @param[in] force force acting on the center of the mass (expressed in the world frame)
   */
  void setExternalForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) final;

  /// should not be used by the users
  void setConstraintForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) final;
  void setGyroscopicMode(GyroscopicMode gyroscopicMode);

  /**
   * Get position of a point on the body
   * @param[in] localIdx this should always be 0 (because the single body object only has one body)
   * @param[in] pos_b the position of the body
   * @param[out] pos_w the corresponding position in the world
   */
  void getPosition(size_t localIdx, const Vec<3>& pos_b, Vec<3>& pos_w) const final;

  /**
   * Get the geometric center of the object
   * @param[out] pos_w the geometric center
   */
  void getPosition(Vec<3>& pos_w) { pos_w = bodyPosition_; };
  void getVelocity(size_t localIdx, Vec<3>& vel_w) const final { vel_w = linVel_; }

  void getVelocity(size_t localIdx, const Vec<3>& pos_b, Vec<3>& vel_w) const final {
    vel_w = linVel_;
    Vec<3> joint2Point_W;
    matvecmul(bodyRotationMatrix_, pos_b, joint2Point_W);
    crossThenAdd(angVel_, joint2Point_W, vel_w);
  }

  void preContactSolverUpdate1(const Vec<3> &gravity, double dt) final;
  void preContactSolverUpdate2(const Vec<3> &gravity, double dt, contact::ContactProblems& problems) final;
  void integrate(double dt, const World* world) final;
  void getContactPointVel(size_t pointId, Vec<3> &vel) const final;

  void updateCollision() override;

  /**
   * Set the linear damping that the object experiences due to air
   * @param[in] damping the damping coefficient in the body frame
   */
  void setLinearDamping(double damping);

  /**
   * Set the angular damping that the object experiences due to air (proportional to the angular velocity).
   * @param[in] damping the damping coefficient in the body frame
   */
  void setAngularDamping(Vec<3> damping);

  /**
   * set the body type. Dynamic means that object is free to move.
   * Kinematic means that the object can have velocity but has an infinite mass (like a conveyor belt).
   * Static means that the object cannot move and has an infinite mass.
   * @param[in] type the body type
   */
  virtual void setBodyType(BodyType type);

  /**
   * get the current collision group of the object. Read the "Contact and Collision" to learn what the collision group is.
   * @return the collision group
   */
  CollisionGroup getCollisionGroup();

  /**
   * get the current collision mast of the object. Read the "Contact and Collision" to learn what the collision mast is.
   * @return the collision mast
   */
  CollisionGroup getCollisionMask();


  /**
   * set the appearance of the object. This works in both RaisimUnity and RaisimUnreal.
   * But depending on the visualizer, they might do different things.
   * You can specify the color by name like "blue", "green", "red"
   * You can also specify the color by a string like "0, 0.5, 0.5, 1.0", which represent the RGBA values
   * @param[in] appearance the appearance of the object
   */
  void setAppearance(const std::string& appearance) { appearance_ = appearance; }

  /**
   * get the current appearance of the object
   * @return appearance
   */
  [[nodiscard]] const std::string& getAppearance() const { return appearance_; }

  /**
   * delete all external forces and torques specified on the object.
   * This method is called at the end of every frame.
   */
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

  void addContactPointVel2(size_t pointId, Vec<3>& vel) final;
  void subContactPointVel2(size_t pointId, Vec<3>& vel) final;

  void updateGenVelWithImpulse(size_t pointId, const Vec<3>& imp) final;
  void updateTimeStep(double dt) final {};
  void updateTimeStepIfNecessary(double dt) final {};
  void updateOrientation();
  void eulerIntegrate(double dt,
                      const Mat<3,3>& initialRotMat,
                      const Vec<3>& initialAngVel,
                      Vec<3>& finalAngVel);


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

  /// bullet frame orientation always coincides with body frame
  Vec<3> colPosition_;

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
