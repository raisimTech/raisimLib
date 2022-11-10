//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_LOADERS_HPP
#define RAISIM_LOADERS_HPP

#include "ArticulatedSystem.hpp"
#include <map>

namespace raisim {

namespace object {
class ArticulatedSystem;
}

namespace urdf {

struct UrdfMaterial {
  UrdfMaterial() {
    color = {0.7, 0.7, 0.7, 1.};
  }
  std::string name;
  Vec<4> color;
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
    scale = {1., 1., 1.};
    color = {0.7, 0.7, 0.7, -1.0};
  };

  Shape::Type shape;
  std::string fileName;
  Vec<3> origin;
  Mat<3, 3> rot;
  Vec<3> scale;
  Vec<4> param;
  raisim::Vec<4> color;
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
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
    limit[0] = std::numeric_limits<double>::lowest();
    limit[1] = std::numeric_limits<double>::max();
    rot.setIdentity();
    springMountPos.setZero();
  }
  std::string name, parent, child;
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
  double velocity_limit = 1e6;
  Vec<4> springMountPos;
};

struct UrdfLink {
  std::string name;
  UrdfJoint parentJoint;
  UrdfLink *parent = nullptr;
  std::vector<UrdfLink *> child;
  std::vector<UrdfBody> visual, collision;
  UrdfLinkInertial inertial;
  Vec<4> color_;
  std::vector<std::shared_ptr<Sensor>> sensor;
};

class LoadFromURDF2 {

 public:
  LoadFromURDF2(ArticulatedSystem &system,
                const std::string& filePath,
                std::vector<std::string> jointOrder,
                bool isItAFilePath);
 private:
  void processLinkFromUrdf(const UrdfLink *urdfLink,
                           Child &raiLink,
                           const std::vector<std::string> &jointsOrder);
  std::map<std::string, UrdfMaterial> mats;
  std::string currentObject_;

};
}

namespace mjcf {

struct MjcfCompilerSetting {
  std::string angle;
  std::string eulerseq;
};

class LoadFromMjcf {
 public:
  void load(ArticulatedSystem &sys,
            RaiSimTinyXmlWrapper &c,
            const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaultDefaultNode,
            const std::unordered_map<std::string, std::pair<std::string, Vec < 3>> >& mesh,
            const MjcfCompilerSetting& setting);


  template<typename T>
  static bool readFromDefault(const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaults,
                              const std::string &className,
                              const std::string &typeName,
                              const std::string &attName,
                              T &value) {
    auto classNode = defaults.at(className).getChildrenMust(typeName)[0];
    if(classNode.getAttributeIfExists(attName, value)) {
      return true;
    } else {
      if(className == "default") {
        return false;
      } else {
        std::string parent = defaults.at(className).template getAttributeMust<std::string>("parent");
        return readFromDefault(defaults, parent, typeName, attName, value);
      }
    }
  }

  template<typename T>
  static bool getParameter(const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaults,
                           const RaiSimTinyXmlWrapper &node,
                           const std::string &typeName,
                           const std::string &attName,
                           T &value) {
    bool result = false;
    std::string className;
    if (node.getAttributeIfExists("class", className)) {
      if (defaults.find(className) == defaults.end())
        node.errorMessage("class " + className + " not found in the default tag");

      result = readFromDefault(defaults,className,typeName,attName,value);
    } else {
      result = defaults.at(typeName).getAttributeIfExists(attName, value);
    }
    return node.getAttributeIfExists(attName, value) || result;
  }

  static void getPoseAndParam(const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaults,
                              const RaiSimTinyXmlWrapper &node,
                              Shape::Type type,
                              const std::string &typeName,
                              Vec<4> &param,
                              Mat<3, 3> &rot,
                              Vec<3> &pos,
                              const MjcfCompilerSetting& setting);

  static void getMjcfSizeParam(const RaiSimTinyXmlWrapper &g, Shape::Type type, Vec<4> &param);
  static void getMjcfPos(const RaiSimTinyXmlWrapper &g, Vec<3> &pos);
  static void posFromFromTo(const RaiSimTinyXmlWrapper &g, Vec<3> &pos);
  static void getMjcfOrientation(const RaiSimTinyXmlWrapper &g, Mat<3, 3> &rot, const std::string& eulerseq, const std::string& anglerep);
  static bool getColorFromMaterial(const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaults,
                                   const std::string &c,
                                   std::string& color);

 private:
  static void processBody(Child &child,
                          Mat<3, 3> &parentBodyRot,
                          Vec<3> &parentBodyPos,
                          const std::string &defaultName,
                          bool isRoot,
                          RaiSimTinyXmlWrapper &c,
                          const std::unordered_map<std::string, RaiSimTinyXmlWrapper> &defaultDefaultNode,
                          const std::unordered_map<std::string, std::pair<std::string, Vec<3>>>& meshAsset,
                          const MjcfCompilerSetting& setting);
};

}

} // namespace raisim
#endif //RAISIM_LOADERS_HPP
