//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_MATERIALS_HPP
#define RAISIM_MATERIALS_HPP
#include <string>
#include <vector>
#include <unordered_map>
#include "raisim_message.hpp"
#include "raisim/RaiSimTinyXmlWrapper.hpp"
#include "raisim/RaiSimTinyXmlWriter.hpp"

namespace raisim {

class World;

struct MaterialPairProperties {
  MaterialPairProperties() = default;
  MaterialPairProperties(double c_f_, double c_r_, double r_th_) {
    c_f = c_f_;
    c_r = c_r_;
    r_th = r_th_;
    c_static_f = c_f_;
    v_static_speed = 0.;
  }

  MaterialPairProperties(double c_f_, double c_r_, double r_th_, double c_sf_, double v_sf_) {
    c_f = c_f_;
    c_r = c_r_;
    r_th = r_th_;
    c_static_f = c_sf_;
    v_static_speed = v_sf_;
    v_static_speed_inv = 1./v_sf_;
  }

  double c_f = 0.8; // coefficient of friction
  double c_r = 0.0; // coefficient of restitution
  double r_th = 0.01; // restitution threshold
  double c_static_f = 0.8;
  double v_static_speed = 0., v_static_speed_inv;
};

class MaterialManager {

 public:
 friend class raisim::World;

  MaterialManager() = default;

  /// upload material data from file
  explicit MaterialManager(const std::string &xmlFile);

  void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold,
                           double staticFriction,
                           double staticFrictionVelocityThreshold);

  inline void setMaterialPairProp(const std::string &mat1,
                           const std::string &mat2,
                           double friction,
                           double restitution,
                           double resThreshold) {
    setMaterialPairProp(mat1, mat2, friction, restitution, resThreshold, friction, 1e-3);
  }

  const MaterialPairProperties &getMaterialPairProp(const std::string &mat1, const std::string &mat2) const;

  void setDefaultMaterialProperties (double friction, double restitution, double resThreshold, double staticFriction, double staticFrictionVelocityThreshold);

 protected:

  void init(const RaiSimTinyXmlWrapper& materialNode);

 public:

  std::unordered_map<unsigned int, MaterialPairProperties> materials_;
  std::unordered_map<std::string, unsigned int> materialKeys_;
  MaterialPairProperties defaultMaterial_;
  unsigned int nextMaterialIdx_ = 0;

};
}

#endif //RAISIM_MATERIALS_HPP
