//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_WIRE_HPP
#define RAISIM_WIRE_HPP

#include <raisim/object/Object.hpp>
#include "Constraints.hpp"
#include "raisim/contact/Contact.hpp"

namespace raisim {

class LengthConstraint : public Constraints {

 public:
  enum class WireType : int {
    STIFF = 0,
    COMPLIANT,
    CUSTOM
  };

  enum class StretchType : int {
    STRETCH_RESISTANT_ONLY = 0,
    COMPRESSION_RESISTANT_ONLY,
    BOTH
  };

  LengthConstraint(Object *obj1, size_t localIdx1, Vec<3> pos1_b, Object *obj2, size_t localIdx2, Vec<3> pos2_b, double length);
  virtual ~LengthConstraint() = default;

  /**
   * update internal variables (called by World::integrate1()) */
  void update(contact::ContactProblems& contact_problems);

  /**
   * @return the length of the wire */
  double getLength() const;

  /**
   * get wire width for visualization
   * @return width
   */
  double getVisualizationWidth() { return width_; }

  /**
   * set visualization width
   * @param width width
   */
  void setVisualizationWidth(double width) { width_ = width; }

  /**
   * @return the distance between the two mounting points */
  double getDistance() const;

  /**
   * @return the first attachment point in the World frame */
  const Vec<3>& getP1() const;

  /**
   * @return the second attachment point in the World frame */
  const Vec<3>& getP2() const;

  /**
   * @return the first object to which the wire is attached */
  Object *getBody1() const;

  /**
   * @return the second object to which the wire is attached */
  Object *getBody2() const;

  /**
   * @return the direction of the normal (i.e., p2-p1 normalized) */
  const Vec<3>& getNorm() const;

  /**
   * @return the local index of object1 */
  size_t getLocalIdx1() const;

  /**
   * @return the local index of object2 */
  size_t getLocalIdx2() const;

  /**
   * @return the stretch length (i.e., constraint violation) */
  double getStretch() const;

  /**
   * set name of the wire
   * @param name the constraint's name */
  void setName(const std::string& name) {
    name_ = name;
  }

  /**
   * get name of the wire
   * @return the wire name */
  const std::string& getName() const {
    return name_;
  }

  /**
   * set stretch type of the wire
   * Check http://raisim.com/sections/StiffLengthConstraint.html
   * @param type the wire stretch type. Available types: STRETCH_RESISTANT_ONLY, COMPRESSION_RESISTANT_ONLY, BOTH */
  void setStretchType(StretchType type) { stretchType_ = type; }

  /**
   * get the constraint's stretch type
   * @return the wire stretch type */
  StretchType getStretchType() const { return stretchType_; }

  /**
   * get wire type
   * @return wire type */
  WireType getWireType() const { return type_; }

  /**
   * get pos on ob1 where the wire is attached
   * @return mount position 1
   */
  const Vec<3>& getOb1MountPos() const;

  /**
   * get pos on ob2 where the wire is attached
   * @return mount position 2
   */
  const Vec<3>& getOb2MountPos() const;

 protected:
  Vec<3> position1_;
  Object* body1_;
  Vec<3> position2_;
  Object* body2_;
  Vec<3> p1_w_;
  size_t localIdx1_;
  Vec<3> p2_w_;
  size_t localIdx2_;
  Vec<3> norm_;
  double length_;
  double distance_;
  double stretch_;
  double width_ = 0.02; /// for visualization only
  WireType type_;
  StretchType stretchType_ = StretchType::COMPRESSION_RESISTANT_ONLY;

  virtual void applyTension(contact::ContactProblems& contact_problems) = 0;

 public:
  bool isActive = false;

};

}

#endif //RAISIM_WIRE_HPP
