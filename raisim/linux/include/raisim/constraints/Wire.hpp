//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_WIRE_HPP
#define RAISIM_WIRE_HPP

#include <raisim/object/Object.hpp>
#include "Constraints.hpp"

namespace raisim {

class Wire : public Constraints {

 public:
  Wire(Object *obj1, size_t localIdx1, Vec<3> pos1_b, Object *obj2, size_t localIdx2, Vec<3> pos2_b, double length);

  /* update internal variables (called by integrate1())*/
  void update();

  /* return the length of the wire */
  double getLength();

  /* return the distance between the two mounting points */
  double getDistance();

  /* return the first attachment point in the World frame */
  const Vec<3>& getP1();

  /* return the second attachment point in the World frame */
  const Vec<3>& getP2();

  /* return the first object to which the wire is attached */
  Object *getBody1();

  /* return the second object to which the wire is attached */
  Object *getBody2();

  /* return the direction of the normal (i.e., p2-p1 normalized) */
  const Vec<3>& getNorm();

  /* return the local index of object1 */
  size_t getLocalIdx1();

  /* return the local index of object2 */
  size_t getLocalIdx2();

  /* return the stretch length (i.e., constraint violation) */
  double getStretch();

  /* set name of the wire*/
  void setName(const std::string& name) {
    name_ = name;
  }

  /* get name of the wire*/
  const std::string& getName() {
    return name_;
  }

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
  double dampedStretch_;

 public:
  bool isActive = false;

};

}

#endif //RAISIM_WIRE_HPP
