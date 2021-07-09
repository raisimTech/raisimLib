//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_HEIGHTMAP_HPP
#define RAISIM_HEIGHTMAP_HPP

#include <raisim/object/singleBodies/SingleBodyObject.hpp>
#include <raisim/Terrain.hpp>

namespace raisim {

class HeightMap final : public SingleBodyObject {

 public:

  /// loading heightmap from a text file
  HeightMap(double centerX,
            double centerY,
            const std::string& raisimHeightMapFileName);

  HeightMap(double centerX,
            double centerY,
            const std::string& pngFileName,
            double xSize,
            double ySize,
            double heightScale,
            double heightOffset);

  HeightMap(size_t xSamples,
            size_t ysamples,
            double xSize,
            double ySize,
            double centerX,
            double centerY,
            const std::vector<double> &height);

  HeightMap(size_t xSamples,
            int ysamples,
            double xSize,
            double ySize,
            double centerX,
            double centerY,
            const Eigen::VectorXd &heightIn) :
      SingleBodyObject(HEIGHTMAP) {
    height_.reserve(heightIn.rows());
    for(size_t i=0; i<heightIn.rows(); i++)
      height_.push_back(heightIn[i]);
    init(xSamples, ysamples, xSize, ySize, centerX, centerY);
  }

  HeightMap(double xSize,
            double ySize,
            double centerX,
            double centerY,
            const Eigen::MatrixXd &heightIn) :
      SingleBodyObject(HEIGHTMAP) {
    height_.reserve(heightIn.rows() * heightIn.cols());
    for(size_t i=0; i<heightIn.rows(); i++)
        for(size_t j=0; i<heightIn.cols(); j++)
          height_.push_back(heightIn(i,j));
    init(heightIn.rows(), heightIn.cols(), xSize, ySize, centerX, centerY);
  }

  HeightMap(double centerX, double centerY, const TerrainProperties &terrainProperties);

  void init(size_t xSamples, size_t ysamples, double xSize, double ySize, double centerX, double centerY);
  std::vector<double> &getHeightMap();
  const std::vector<double> &getHeightMap() const;

  double getHeight(double x, double y) const;
  void getNormal(double x, double y, Vec<3>& normal) const;

  void destroyCollisionBodies(dSpaceID id) final;

  size_t getXSamples() const { return xSamples_; }
  size_t getYSamples() const { return ySamples_; }

  double getXSize() const { return sizeX_; }
  double getYSize() const { return sizeY_; }

  double getCenterX() const { return centerX_; }
  double getCenterY() const { return centerY_; }

  const std::vector<double> &getHeightVector() const { return height_; }

  ~HeightMap() final;

  void updateCollision() final;

  void setPosition(const Eigen::Vector3d &originPosition) final {
    SingleBodyObject::setPosition(originPosition);
    updateCollision();
  }

  void setPosition(double x, double y, double z) {
    SingleBodyObject::setPosition(x, y, z);
    updateCollision();
  }

  void setPosition(const Vec<3>& pos) {
    SingleBodyObject::setPosition(pos);
    updateCollision();
  }

  void setOrientation(const Eigen::Quaterniond &quaternion) {
    SingleBodyObject::setOrientation(quaternion);
    updateCollision();
  }

  void setOrientation(const Eigen::Vector4d &quaternion) {
    SingleBodyObject::setOrientation(quaternion);
    updateCollision();
  }

  void setOrientation(double w, double x, double y, double z) {
    SingleBodyObject::setOrientation(w, x, y, z);
    updateCollision();
  }

  void setOrientation(const Eigen::Matrix3d &rotationMatrix) {
    SingleBodyObject::setOrientation(rotationMatrix);
    updateCollision();
  }

 private:
  void generateTerrain(const TerrainProperties &terrainProperties);
  std::vector<double> height_;
  dHeightfieldDataID heightFieldData_;
  double centerX_, centerY_, sizeX_, sizeY_;
  size_t xSamples_, ySamples_;
  Mat<3,3> rotOffset_;

};

}

#endif //RAISIM_HEIGHTMAP_HPP
