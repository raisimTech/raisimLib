//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SERVER_VISUALS_HPP_
#define RAISIM_INCLUDE_RAISIM_SERVER_VISUALS_HPP_

#include "raisim/World.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace raisim {

class RaisimServer;

struct PolyLine {
  friend class raisim::RaisimServer;

  std::string name;
  Vec<4> color = {1, 1, 1, 1};
  std::vector<Vec<3>> points;
  double width = 0.01;
  /**
   * @param[in] r red value of the color (max=1).
   * @param[in] g green value of the color (max=1).
   * @param[in] b blue value of the color (max=1).
   * @param[in] a alpha value of the color (max=1).
   * set the color of the polyline. */
  void setColor(double r, double g, double b, double a) { color = {r, g, b, a}; }

  /**
   * @param[in] point new polyline point.
   * append a new point to the polyline. */
  void addPoint(const Eigen::Vector3d &point) { points.push_back(point); }

  /**
   * clear all polyline points. */
  void clearPoints() { points.clear(); }

 protected:
  uint32_t visualTag = 0;
};

struct ArticulatedSystemVisual {
  friend class raisim::RaisimServer;

  ArticulatedSystemVisual(const std::string &urdfFile) : obj(urdfFile) {
    color.setZero();
  }

  ~ArticulatedSystemVisual() = default;

  /**
   * @param[in] r red value (max=1)
   * @param[in] g green value (max=1)
   * @param[in] b blue value (max=1)
   * @param[in] a alpha value (max=1)
   * set color. if the alpha value is 0, it uses the original color defined in the mesh file */
  void setColor(double r, double g, double b, double a) {
    color = {r, g, b, a};
  }

  /**
   * @param[in] gc the generalized coordinate
   * set the configuration of the visualized articulated system */
  void setGeneralizedCoordinate(const Eigen::VectorXd &gc) {
    obj.setGeneralizedCoordinate(gc);
  }

  raisim::Vec<4> color;
  ArticulatedSystem obj;
 protected:
  uint32_t visualTag = 0;
  std::string name;
};

struct Visuals {
  friend class raisim::RaisimServer;
  Shape::Type type;
  std::string name;
  std::string material;
  bool glow = true;
  bool shadow = false;

  // {r, g, b, a}
  Vec<4> color = {1, 1, 1, 1};

  /*
   * sphere     {radius, 0, 0}
   * box        {xlength, ylength, zlength}
   * cylinder   {radius, length, 0}
   * capsule    {radius, length, 0}
   * mesh       {xscale, yscale, zscale}
   */
  Vec<4> size = {0, 0, 0, 0};

  /**
   * @param[in] radius the raidus of the sphere.
   * set size of the sphere. */
  void setSphereSize(double radius) { size[0] = radius; }

  /**
   * @param[in] x length.
   * @param[in] y width.
   * @param[in] z height.
   * set size of the box. */
  void setBoxSize(double x, double y, double z) { size = {x, y, z}; }

  /**
   * @param[in] radius the raidus of the cylinder.
   * @param[in] height the height of the cylinder.
   * set size of the cylinder. */
  void setCylinderSize(double radius, double height) { size = {radius, height, 1.}; }

  /**
   * @param[in] radius the raidus of the capsule.
   * @param[in] height the height of the capsule.
   * set size of the capsule. */
  void setCapsuleSize(double radius, double height) { size = {radius, height, 1.}; }

  /**
   * @param[in] x x coordinate of the visual object.
   * @param[in] y y coordinate of the visual object.
   * @param[in] z z coordinate of the visual object.
   * set the position of the visual object. */
  void setPosition(double x, double y, double z) { position = {x, y, z}; }

  /**
   * @param[in] w angle part of the quaternion.
   * @param[in] x scaled x coordinate of the rotation axis.
   * @param[in] y scaled y coordinate of the rotation axis.
   * @param[in] z scaled z coordinate of the rotation axis.
   * set the orientation of the visual object. */
  void setOrientation(double w, double x, double y, double z) { quaternion = {w, x, y, z}; }

  /**
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * set the position of the visual object. */
  void setPosition(const Eigen::Vector3d &pos) { position = pos; }

  /**
   * @param[in] ori quaternion of the visual object in Eigen::Vector4d.
   * set the orientation of the visual object. */
  void setOrientation(const Eigen::Vector4d &ori) { quaternion = ori; }

  /**
   * @param[in] r red value of the color (max=1).
   * @param[in] g green value of the color (max=1).
   * @param[in] b blue value of the color (max=1).
   * @param[in] a alpha value of the color (max=1).
   * set the color of the visual object. */
  void setColor(double r, double g, double b, double a) { color = {r, g, b, a}; }

  /**
   * @return the position of the visual object.
   * get the position of the visual object. */
  Eigen::Vector3d getPosition() { return position.e(); }

  /**
   * @return the orientation of the visual object.
   * get the orientation of the visual object. */
  Eigen::Vector4d getOrientation() { return quaternion.e(); }

 private:
  Vec<3> position = {0, 0, 0};
  Vec<4> quaternion = {1, 0, 0, 0};

 protected:
  uint32_t visualTag = 0;

};

struct VisualMesh : public Visuals {
  friend class raisim::RaisimServer;

  void updateMesh (const std::vector<float>& vertexArray,
                   const std::vector<uint8_t>& colorArray) {
    updateMesh_ = true;
    vertexArray_ = vertexArray;
    colorArray_ = colorArray;
  }

  [[nodiscard]] bool isUpdated () const {
    return updateMesh_;
  };

 protected:
  std::string meshFileName_;
  std::vector<float> vertexArray_;
  std::vector<uint8_t> colorArray_;
  std::vector<int32_t> indexArray_;
  bool updateMesh_ = false;
};


struct InstancedVisuals {
  friend class raisim::RaisimServer;

  InstancedVisuals(Shape::Type type,
                   std::string name,
                   const Vec<3>& size,
                   const Vec<4>& color1,
                   const Vec<4>& color2) {
    this->size = size;
    this->color1 = color1;
    this->color2 = color2;
    this->name = std::move(name);
    this->type = type;
  }
  /**
   * return the number of instances
   */
  size_t count() { return data.size(); }

  /**
   * clear the instances
   */
  void clearInstances() {
    data.clear();
  }

  void resize(size_t numberOfObjects) {
    data.resize(numberOfObjects);
    for (auto& d: data) {
      d.quat = Vec<4>{1, 0, 0, 0};
      d.scale = Vec<3>{1, 1, 1};
      d.scale = size;
    }
  }

  /**
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * @param[in] ori quaternion of the visual object in Eigen::Vector4d.
   * @param[in] scale scale of the visual object in Eigen::Vector3d.
   * @param[in] colorWeight the final color is an weighted average of color1 and color2
   * add a new instance of the specified pose */
  void addInstance(const Eigen::Vector3d &pos, const Eigen::Vector4d &ori, const Eigen::Vector3d& scale, float colorWeight = 0.f) {
    data.emplace_back();
    data.back().pos = pos;
    data.back().quat = ori;
    data.back().scale = {size[0]*scale[0], size[1]*scale[1], size[2]*scale[2]};
    data.back().colorWeight = colorWeight;
  }

  /**
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * @param[in] ori quaternion of the visual object in Eigen::Vector4d.
   * @param[in] colorWeight the final color is an weighted average of color1 and color2
   * add a new instance of the specified pose */
    void addInstance(const Eigen::Vector3d &pos, const Eigen::Vector4d &ori, float colorWeight = 0.f) {
      data.emplace_back();
      data.back().pos = pos;
      data.back().quat = ori;
      data.back().scale = size;
      data.back().colorWeight = colorWeight;
    }

  /**
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * add a new instance of the specified position and an identity rotation */
  void addInstance(const Eigen::Vector3d &pos, float colorWeight = 0.f) {
    data.emplace_back();
    data.back().pos = pos;
    data.back().quat = raisim::Vec<4>{1,0,0,0};
    data.back().scale = size;
    data.back().colorWeight = colorWeight;
  }

  /**
   * remove a single instance */
  void removeInstance(size_t id) {
    data[id] = data.back();
    data.pop_back();
  }

  /**
   * @param[in] id instance id.
   * @param[in] pos position of the visual object in Eigen::Vector3d.
   * set the position of the specified instance of the visual object. */
  void setPosition(size_t id, const Eigen::Vector3d &pos) { data[id].pos = pos; }

  /**
   * @param[in] id instance id.
   * @param[in] ori quaternion of the visual object in Eigen::Vector4d.
   * set the orientation of the specified instance of the visual object. */
  void setOrientation(size_t id, const Eigen::Vector4d &ori) { data[id].quat = ori; }

  /**
   * @param[in] color1 first color.
   * @param[in] color2 second color.
   * The color of the instance is interpolated between the two colors.
   * 100% color1 when weight is 0.
   * */
  void setColor(const Vec<4>& color1, const Vec<4>& color2) {
    this->color1 = color1;
    this->color2 = color2; }

  /**
   * @param[in] id instance id
   * @param[in] weight the color weight
   * The actual color is linearly interpolated between color1 and color2 which are set by setColor method.
   * 100% color1 when weight is 0.
   */
  void setColorWeight(size_t id, float weight) {
    data[id].colorWeight = weight;
  }

  /**
   * @return the position of the visual object.
   * get the position of the specified instance of the visual object. */
  Eigen::Vector3d getPosition(size_t id) { return data[id].pos.e(); }

  /**
   * @return the orientation of the visual object.
   * get the orientation of the specified instance of the visual object. */
  Eigen::Vector4d getOrientation(size_t id) { return data[id].quat.e(); }

  /**
   * @return the orientation of the visual object.
   * get the orientation of the specified instance of the visual object. */
  Eigen::Vector3d getScale(size_t id) { return data[id].scale.e(); }

 protected:
  struct PerInstanceData {
    Vec<3> pos;
    Vec<4> quat;
    Vec<3> scale;
    float colorWeight;
  };
  Vec<3> size;
  std::vector<PerInstanceData> data;
  Shape::Type type;
  std::string name;
  std::string material;
  std::string meshFileName;

  // {r, g, b, a}
  Vec<4> color1 = {1, 1, 1, 1};
  Vec<4> color2 = {1, 1, 1, 1};

  uint32_t visualTag = 0;
};

}


#endif //RAISIM_INCLUDE_RAISIM_SERVER_VISUALS_HPP_
