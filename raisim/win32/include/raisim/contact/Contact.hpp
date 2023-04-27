//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CONTACT_HPP
#define RAISIM_CONTACT_HPP

#include <raisim/configure.hpp>
#include "raisim/helper.hpp"
#include <raisim/Materials.hpp>
#include <ode/objects.h>


namespace raisim {
class World;
class Object;
class SingleBodyObject;
class ArticulatedSystem;
}

namespace raisim {
class Contact {
 public:
  friend class raisim::World;
  friend class raisim::ArticulatedSystem;
  friend class raisim::Object;
  friend class raisim::SingleBodyObject;

  Contact() = default;

  explicit Contact(const Vec<3> &position,
                   const Vec<3> &normal,
                   const Mat<3, 3> & frame,
                   bool objectA,
                   size_t contactProblemIndex,
                   size_t contactIndexInObject,
                   size_t pairObjectIndex,
                   BodyType pairObjectBodyType,
                   size_t pairContactIndexInPairObject,
                   size_t localBodyIndex,
                   double depth,
                   dGeomID colA,
                   dGeomID colB)
      : position_(position),
        normal_(normal),
        frame_(frame),
        depth_(depth),
        contactIndexInObject_(contactIndexInObject),
        pairObjectIndex_(pairObjectIndex),
        pairContactIndexInPairObject_(pairContactIndexInPairObject),
        contactProblemIndex_(contactProblemIndex),
        localBodyIndex_(localBodyIndex),
        objectA_(objectA),
        pairObjectBodyType_(pairObjectBodyType),
        colA_(colA),
        colB_(colB) { }

  ~Contact() = default;

  static inline void computeFrame(const Vec<3>& zAxis, Mat<3,3>& frame) {

    // set contact frame
    // contact frame is R_PiW not R_WPi
    raisim::Vec<3> xAxis;
    raisim::Vec<3> yAxis;

    if (zAxis[0] == 0.0 && zAxis[1] == 0.0)
      xAxis = {1.0, 0.0, 0.0};
    else
      xAxis = {zAxis[1], -zAxis[0], 0};
    cross(zAxis, xAxis, yAxis);

    const double xNormInv = 1. / xAxis.norm();
    const double yNormInv = 1. / yAxis.norm();
    const double zNormInv = 1. / zAxis.norm();

    frame[0] = xAxis[0] * xNormInv;
    frame[1] = yAxis[0] * yNormInv;
    frame[2] = zAxis[0] * zNormInv;
    frame[3] = xAxis[1] * xNormInv;
    frame[4] = yAxis[1] * yNormInv;
    frame[5] = zAxis[1] * zNormInv;
    frame[6] = xAxis[2] * xNormInv;
    frame[7] = yAxis[2] * yNormInv;
    frame[8] = zAxis[2] * zNormInv;
  }

public:

  /**
   * the contact position
   * @return the contact position
   */
  [[nodiscard]] const Vec<3> &getPosition() const {
    return position_;
  }

  /**
   * the contact normal vector pointing to ObjectA
   * @return frame
   */
  [[nodiscard]] const Vec<3> &getNormal() const {
    return normal_;
  }

  /**
   * returns a TRANSPOSE of the frame that the impulse is expressed.
   * @return contact frame
   */
  [[nodiscard]] const Mat<3, 3> &getContactFrame() const {
    return frame_;
  }

  /**
   * returns the corresponding index in raisim::World::getContactProblem
   * @return contact index
   */
  [[nodiscard]] size_t getIndexContactProblem() const {
    return contactProblemIndex_;
  }

  /**
   * returns the corresponding index in raisim::Object::getContacts
   * @return contact index
   */
  [[nodiscard]] size_t getIndexInObjectContactList() const {
    return contactIndexInObject_;
  }

  /**
   * returns the contacting object index in raisim::World::getObjectList
   * @return object index
   */
  [[nodiscard]] size_t getPairObjectIndex() const {
    return pairObjectIndex_;
  }

  /**
   * returns the contact index in the contacting (the paired) object in raisim::Object::getContacts
   * This does not work if the pair body is STATIC because contacts on static bodies are not stored.
   * First check the pair body type with getPairObjectBodyType
   * @return contact index
   */
  [[nodiscard]] size_t getPairContactIndexInPairObject() const {
    return pairContactIndexInPairObject_;
  }

  /**
   * returns the impulse. You have to divide this number by the time step to get the force
   * @return impulse
   */
  [[nodiscard]] const Vec<3> &getImpulse() const {
    return *impulse_;
  }

  /**
   * returns if the object is objectA. When there is a contact, the two paired objects are assigned to be objectA and objectB arbitrarily.
   * The contact frame is defined such that its z-axis is pointing towards the objectA.
   * The contact impulse is defined as an external force that objectA experiences.
   * @return impulse
   */
  [[nodiscard]] bool isObjectA() const {
    return objectA_;
  }

  /**
   * returns the body type of the paired object. Please read https://raisim.com/sections/Object.html#body-types to learn about it
   * @return the body type
   */
  [[nodiscard]] BodyType getPairObjectBodyType() const {
    return pairObjectBodyType_;
  }

  /**
   * returns the inverse apparent inertia of the contacting point
   * @return inertia
   */
  [[nodiscard]] Mat<3, 3> &getInvInertia() {
    return Minv_;
  }

  /**
   * get local body index of this contact. The local index is assigned to each moving body in an object
   * @return the body index.
   */
  [[nodiscard]] size_t getlocalBodyIndex() const {
    return localBodyIndex_;
  }

  /**
   * The penetration depth of the contact
   * @return the depth of the contact
   */
  [[nodiscard]] double getDepth() const {
    return depth_;
  }

  /**
   * returns if the contact is self collision
   * @return if this is a self collision
   */
  [[nodiscard]] bool isSelfCollision() const {
    return isSelfCollision_;
  }

  /**
   * this is set true for one self-collision point so that you don't count them twice
   * @return if you can skip this contact point while iterating contact points
   */
  [[nodiscard]] bool skip() const {
    return skip_;
  }

  /**
   * get the collision body of this object. This is ODE interface
   * @return collision body
   */
  [[nodiscard]] dGeomID getCollisionBodyA() {
    return colA_;
  }

  /**
   * get the collision body of the pairing object. This is ODE interface
   * @return collision body
   */
  [[nodiscard]] dGeomID getCollisionBodyB() {
    return colB_;
  }

protected:
  void setSelfCollision() {
    isSelfCollision_ = true;
  }

  void setSkip() {
    skip_ = true;
  }

  double &getImpactVel() {
    return impactVel_;
  }

  void setImpulse(Vec<3> *im) {
    impulse_ = im;
  }



 private:
  Mat<3, 3> frame_;             // contactFrame of A
  Vec<3> position_;             // position of A = position of B
  Vec<3> normal_;               // normal of A (normal of B = - normalA)
  Mat<3, 3> Minv_;               // inverse apparent inertia matrix
  Vec<3> *impulse_;
  double impactVel_;
  double depth_;
  size_t contactIndexInObject_;
  size_t pairObjectIndex_;
  size_t pairContactIndexInPairObject_;
  size_t contactProblemIndex_;
  size_t localBodyIndex_ = 0;
  bool isSelfCollision_ = false;
  bool skip_ = false;
  bool objectA_;                                  // true (A) / false (B)
  BodyType pairObjectBodyType_;
  dGeomID colA_, colB_;

};

} // raisim

#endif //RAISIM_CONTACT_HPP
