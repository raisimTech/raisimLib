//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_LOADERS_HPP
#define RAISIM_LOADERS_HPP

#include "ArticulatedSystem.hpp"
#include <map>

namespace raisim {

inline double firstNumber(const char *txt) {
  double num;
  std::string s(txt);
  std::string delimiter = " ";

  while (s.substr(0, 1) == " ")
    s = s.substr(1, s.size());
  std::string token = s.substr(0, s.find(delimiter));
  num = std::stod(token);

  return num;
}

inline Vec<3> char2Vec3(const char *txt) {
  Vec<3> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 3; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 2) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<2> char2Vec2(const char *txt) {
  Vec<2> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 2; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 1) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<6> char2Vec6(const char *txt) {
  Vec<6> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 6; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());

    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 5) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

inline Vec<4> char2Vec4(const char *txt) {
  Vec<4> vec;
  std::string s(txt);
  std::string delimiter = " ";

  for (int i = 0; i < 4; i++) {
    while (s.substr(0, 1) == " ")
      s = s.substr(1, s.size());
    std::string token = s.substr(0, s.find(delimiter));
    vec[i] = std::stod(token);
    if (i != 3) s.erase(0, s.find(delimiter) + delimiter.length());
  }
  return vec;
}

namespace object {
class ArticulatedSystem;
}

namespace urdf {

struct UrdfMaterial {
  UrdfMaterial() {
    color = {0.7, 0.7, 0.7};
  }
  std::string name;
  Vec<3> color;
};

inline Shape::Type charToGeom(const std::string &txt) {
  if (txt == "box")
    return Shape::Box;
  else if (txt == "mesh")
    return Shape::Mesh;
  else if (txt == "cylinder")
    return Shape::Cylinder;
  else if (txt == "capsule")
    return Shape::Capsule;
  else if (txt == "sphere")
    return Shape::Sphere;

  return Shape::NotShape;
}

struct UrdfBody {
  UrdfBody() {
    origin.setZero();
    rot.setIdentity();
    scale = {1.,1.,1.};
  };

  Shape::Type shape;
  std::string fileName;
  Vec<3> origin;
  Mat<3, 3> rot;
  Vec<3> scale;
  std::vector<double> param;
  std::string mat;
  std::string collision_mat;
  std::string name;
};

struct UrdfLinkInertial {
  UrdfLinkInertial() {
    inertia.setZero();
    origin.setZero();
    rot.setIdentity();
  }

  Vec<3> origin;
  Mat<3, 3> rot;
  double mass = 0;
  Mat<3, 3> inertia;
};

struct UrdfJoint {
  UrdfJoint() {
    limit.setZero();
    origin.setZero();
    axis[0] = 1; axis[1] = 0; axis[2] = 0;
    limit[0] = std::numeric_limits<double>::lowest();
    limit[1] = std::numeric_limits<double>::max();
    rot.setIdentity();
    springMountPos.setZero();
  }
  std::string name = "", parent, child;
  Joint::Type type;
  Vec<3> origin;
  Mat<3, 3> rot;
  Vec<3> axis;
  Vec<2> limit;
  double damping = 0;
  double friction = 0;
  double stiffness = 0;
  double rotor_inertia = 0;
  double torque_limit = -1.;
  Vec<4> springMountPos;
};

struct UrdfLink {
  std::string name;
  UrdfJoint parentJoint;
  UrdfLink* parent = nullptr;
  std::vector<UrdfLink *> child;
  std::vector<UrdfBody> visual, collision;
  UrdfLinkInertial inertial;
  Vec<4> color_;
};

class LoadFromURDF2 {

 public:
  LoadFromURDF2(ArticulatedSystem &system, std::string filePath, std::vector<std::string> jointOrder, bool isItAFilePath);
 private:
  void processLinkFromUrdf(const UrdfLink *urdfLink,
                           Child &raiLink,
                           const std::vector<std::string> &jointsOrder);
  std::map<std::string, UrdfMaterial> mats;
  std::string currentObject_;


};
}

namespace mjcf {
class LoadFromMJCF {
 public:
  LoadFromMJCF(ArticulatedSystem &system, std::string filePath, std::vector<std::string> jointOrder);
};
}
}

#endif //RAISIM_LOADERS_HPP
