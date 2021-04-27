//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_CONTACT_HPP
#define RAISIM_CONTACT_HPP

#include <raisim/configure.hpp>
#include "raisim/helper.hpp"

namespace raisim {

typedef std::vector<std::pair<std::vector <raisim::Mat<3, 3>, AlignedAllocator<raisim::Mat<3, 3>, 32>>, raisim::Vec<3>>, AlignedAllocator<std::pair<std::vector <raisim::Mat<3, 3>, AlignedAllocator<raisim::Mat<3, 3>, 32>>, raisim::Vec<3>>, 32>> DelassusType;

class World;

namespace contact {

class Contact {
 public:

  friend class raisim::World;

  explicit Contact(Vec<3> &position,
                   Vec<3> &normal,
                   bool objectA,
                   size_t contactProblemIndex,
                   size_t contactIndexInObject,
                   size_t pairObjectIndex,
                   BodyType pairObjectBodyType,
                   size_t pairContactIndexInPairObject,
                   size_t localBodyIndex,
                   double depth)
    : position_(position), normal_(normal),
      depth_(depth),
      contactIndexInObject_(contactIndexInObject),
      pairObjectIndex_(pairObjectIndex),
      pairContactIndexInPairObject_(pairContactIndexInPairObject),
      contactProblemIndex_(contactProblemIndex),
      localBodyIndex_(localBodyIndex),
      objectA_(objectA),
      pairObjectBodyType_(pairObjectBodyType)
  {
    computeFrame();
  }

  void computeFrame() {

    // set contact frame
    // contact frame is R_PiW not R_WPi
    raisim::Vec<3> &zAxis = normal_;
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

    frame_[0] = xAxis[0] *xNormInv;
    frame_[1] = yAxis[0] *yNormInv;
    frame_[2] = zAxis[0] *zNormInv;
    frame_[3] = xAxis[1] *xNormInv;
    frame_[4] = yAxis[1] *yNormInv;
    frame_[5] = zAxis[1] *zNormInv;
    frame_[6] = xAxis[2] *xNormInv;
    frame_[7] = yAxis[2] *yNormInv;
    frame_[8] = zAxis[2] *zNormInv;
  }

  const Vec<3> &getPosition() const {
    return position_;
  }

  const Vec<3> &getNormal() const {
    return normal_;
  }

  const Mat<3, 3> &getContactFrame() const {
    return frame_;
  }

  size_t getIndexContactProblem() const {
    return contactProblemIndex_;
  }

  size_t getIndexInObjectContactList() const {
    return contactIndexInObject_;
  }

  size_t getPairObjectIndex() const {
    return pairObjectIndex_;
  }

  size_t getPairContactIndexInPairObject() const {
    return pairContactIndexInPairObject_;
  }

  Vec<3> *getImpulse() const {
    return impulse_;
  }

  bool isObjectA() const {
    return objectA_;
  }

  BodyType getPairObjectBodyType() const {
    return pairObjectBodyType_;
  }

  void setImpulse(Vec<3> *impulse) {
    impulse_ = impulse;
  }

  void setInvInertia(Mat<3, 3> *appIinv) {
    appInertiaInv_ = appIinv;
  }

  const Mat<3, 3> *getInvInertia() const {
    return appInertiaInv_;
  }

  size_t getlocalBodyIndex() const {
    return localBodyIndex_;
  }

  double getDepth() const {
    return depth_;
  }

  bool isSelfCollision() const {
    return isSelfCollision_;
  }

  void setSelfCollision() {
    isSelfCollision_ = true;
  }

  bool skip() const {
    return skip_;
  }

  void setSkip() {
    skip_ = true;
  }
  double& getImpactVel() {
    return impactVel_;
  }

 private:
  Mat<3, 3> frame_;              // contactFrame of A
  Vec<3> position_;             // position of A = position of B
  Vec<3> normal_;               // normal of A (normal of B = - normalA)
  Vec<3> *impulse_;
  Mat<3, 3> *appInertiaInv_;
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

};

} // contact
} // raisim

#endif //RAISIM_CONTACT_HPP
