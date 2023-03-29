//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_OBJECT_HPP
#define RAISIM_OBJECT_HPP

#include <memory>
#include <vector>
#include <raisim/contact/BisectionContactSolver.hpp>
#include "raisim/math.hpp"
#include "raisim/contact/Contact.hpp"
#include "raisim/Materials.hpp"
#include "raisim/configure.hpp"

#include "ode/ode.h"

namespace raisim {

class World;
class RaisimServer;

namespace contact {
class BisectionContactSolver;
class Single3DContactProblem;
}

class Object {
  friend class raisim::World;
  friend class raisim::RaisimServer;
  friend class raisim::contact::BisectionContactSolver;
  friend class raisim::contact::Single3DContactProblem;

 public:
  explicit Object();
  virtual ~Object() = default;

 public:
  void clearPerObjectContact();
  void addContactToPerObjectContact(Contact &contact);
  void setIndexInWorld(size_t indexInWorld_);

  /**
   * get the world index. raisim::World::getObjects() returns a vector of object pointers.
   * This is method returns the index of this object in the vector.
   * @return the world index */
  [[nodiscard]] size_t getIndexInWorld() const;

  /**
   * get a vector of all contacts on the object.
   * @return contacts on the body */
  [[nodiscard]] const std::vector<Contact> &getContacts() const;
  [[nodiscard]] std::vector<Contact> &getContacts();

  virtual void updateCollision() = 0;
  virtual void preContactSolverUpdate1(const Vec<3> &gravity, double dt) = 0;
  virtual void preContactSolverUpdate2(const Vec<3> &gravity, double dt, contact::ContactProblems& problems) = 0;
  virtual void integrate(double dt, const World* gravity) = 0;

  /// apply forces at the Center of Mass
  virtual void setExternalForce(size_t localIdx, const Vec<3>& force) = 0;
  /// apply torque on a body
  virtual void setExternalTorque(size_t localIdx, const Vec<3>& torque) = 0;
  /// apply force (expressed in the world frame) at specific location of the body (expressed in the body frame)
  virtual void setExternalForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) = 0;
  /// apply spring force (expressed in the world frame) at specific location of the body (expressed in the body frame)
  // spring force is not visualized
  virtual void setConstraintForce(size_t localIdx, const Vec<3>& pos, const Vec<3>& force) = 0;

  [[nodiscard]] virtual double getMass(size_t localIdx) const = 0;

  /**
   * get the object type.
   * Possible types are SPHERE, BOX, CYLINDER, CONE, CAPSULE, MESH, HALFSPACE, COMPOUND, HEIGHTMAP, ARTICULATED_SYSTEM
   * @return the object type */
  [[nodiscard]] virtual ObjectType getObjectType() const = 0;
  virtual void getPosition(size_t localIdx, Vec<3>& pos_w) const = 0;
  virtual void getVelocity(size_t localIdx, Vec<3>& vel_w) const = 0;
  virtual void getOrientation(size_t localIdx, Mat<3,3>& rot) const = 0;
  virtual void getPosition(size_t localIdx, const Vec<3>& pos_b, Vec<3>& pos_w) const = 0;
  virtual void getVelocity(size_t localIdx, const Vec<3>& pos_b, Vec<3>& vel_w) const = 0;

  /**
   * get the object body type.
   * Available types are: DYNAMIC (movable and finite mass), STATIC (not movable and infinite mass), KINETIC (movable and infinite mass)
   * @return the body type */
  [[nodiscard]] virtual BodyType getBodyType(size_t localIdx) const { return bodyType_; };

  /**
   * get the object body type.
   * Available types are: DYNAMIC (movable and finite mass), STATIC (not movable and infinite mass), KINETIC (movable and infinite mass).
   * @return the body type */
  [[nodiscard]] virtual BodyType getBodyType() const { return bodyType_; };

  /**
   * get the contact point velocity in the world frame.
   * @param[in] pointId the contact index. This is an index of a contact in the contact vector that you can retrieve from getContacts().
   * @param[out] vel the contact point velocity in the world frame */
  virtual void getContactPointVel(size_t pointId, Vec<3> &vel) const = 0;

  /**
   * set the name of the object. You can retrieve an object by name using raisim::World::getObject()
   * @param[in] name name of the object. */
  void setName(const std::string& name) { name_ = name; }

  /**
   * get the name of the object
   * @return name of the object */
  [[nodiscard]] const std::string& getName() const { return name_; }

  // external force visualization
  [[nodiscard]] const std::vector<Vec<3>>& getExternalForce() const { return externalForceViz_; }
  [[nodiscard]] const std::vector<Vec<3>>& getExternalForcePosition() const { return externalForceVizPos_; }
  [[nodiscard]] const std::vector<Vec<3>>& getExternalTorque() const { return externalTorqueViz_; }
  [[nodiscard]] const std::vector<Vec<3>>& getExternalTorquePosition() const { return externalTorqueVizPos_; }
  virtual void clearExternalForcesAndTorques() = 0;

 protected:
  double &getImpactVel(size_t idx);
  virtual void destroyCollisionBodies(dSpaceID id) = 0;
  virtual void addContactPointVel(size_t pointId, Vec<3>& vel) = 0;
  virtual void subContactPointVel(size_t pointId, Vec<3>& vel) = 0;

  virtual void addContactPointVel2(size_t pointId, Vec<3>& vel) = 0;
  virtual void subContactPointVel2(size_t pointId, Vec<3>& vel) = 0;

  virtual void updateTimeStep(double dt) = 0;
  virtual void updateTimeStepIfNecessary(double dt) = 0;
  virtual void updateSensorsIfNecessary(const raisim::World* world) { };
  virtual void updateGenVelWithImpulse(size_t pointId, const Vec<3>& imp) = 0;
  virtual void appendJointLimits(std::vector<contact::Single3DContactProblem, AlignedAllocator<contact::Single3DContactProblem, 32>>& problem) {};
  virtual double enforceJointLimits(contact::Single3DContactProblem& problem) { return 0; };
  virtual void appendConstraints(contact::ContactProblems& problems) {};
  std::vector<Contact> contacts_;
  size_t contactSize_ = 0;

  size_t indexInWorld_;
  BodyType bodyType_ = BodyType::DYNAMIC;
  std::string name_;

  // external force/torque visualization
  std::vector<bool> isExternalForces_;
  std::vector<Vec<3>> externalForceAndTorque_;
  std::vector<Vec<3>> externalForceAndTorquePos_;
  std::vector<size_t> externalForceAndTorqueBodyIndex_;
  std::vector<Vec<3>> externalForceViz_;
  std::vector<Vec<3>> externalForceVizPos_;
  std::vector<Vec<3>> externalTorqueViz_;
  std::vector<Vec<3>> externalTorqueVizPos_;
  std::vector<SparseJacobian> constraintJaco_;
  std::vector<Vec<3>> constraintPos_;
  std::vector<Vec<3>> constraintForce_;
  std::vector<size_t> constraintIdx_;

 protected:
  uint32_t visualTag = 0;
};

} // raisim

#endif //RAISIM_COLLISIONOBJECT_HPP
