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
    for(size_t i=0; i<size_t(heightIn.rows()); i++)
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
    for(int i=0; i<heightIn.rows(); i++)
        for(int j=0; i<heightIn.cols(); j++)
          height_.push_back(heightIn(i,j));
    init(heightIn.rows(), heightIn.cols(), xSize, ySize, centerX, centerY);
  }

  HeightMap(double centerX, double centerY, const TerrainProperties &terrainProperties);

  /**
   * Update the existing heightmap with a new terrain properties.
   * The size should be the same. You cannot change the topology.
   * If you want to create a new topology, delete an old one and make a new heightmap.
   * @param[in] centerX center of the heightmap
   * @param[in] centerY center of the heightmap
   * @param[in] terrainProperties new terrain properties. You cannot change the sample size
   */
  void update(double centerX, double centerY, const TerrainProperties &terrainProperties);

  /**
   * Update the existing heightmap with a new terrain properties.
   * The size should be the same. You cannot change the topology.
   * If you want to create a new topology, delete an old one and make a new heightmap.
   * @param[in] centerX center of the heightmap
   * @param[in] centerY center of the heightmap
   * @param[in] terrainProperties new terrain properties. You cannot change the sample size
   */
  void update(double centerX, double centerY, double sizeX, double sizeY, const std::vector<double> &height);

  void init(size_t xSamples, size_t ysamples, double xSize, double ySize, double centerX, double centerY);
  std::vector<double> &getHeightMap();
  [[nodiscard]] const std::vector<double> &getHeightMap() const;

  /**
   * Get height at a given coordinate
   * @param[in] x x position
   * @param[in] y y position
   * @return height
   */
  [[nodiscard]] double getHeight(double x, double y) const;

  /**
   * Get normal at a given coordinate
   * @param[in] x x position
   * @param[in] y y position
   * @param[out] normal normal vector
   */
  void getNormal(double x, double y, Vec<3>& normal) const;

  void destroyCollisionBodies(dSpaceID id) final;


  /**
   * Get the number of points in x direction
   * @return the number of points
   */
  [[nodiscard]] size_t getXSamples() const { return xSamples_; }

  /**
   * Get the number of points in y direction
   * @return the number of points
   */
  [[nodiscard]] size_t getYSamples() const { return ySamples_; }

  /**
   * Get the size of the heightmap in x direction
   * @return the length
   */
  [[nodiscard]] double getXSize() const { return sizeX_; }

  /**
   * Get the number of the heightmap in y direction
   * @return the width
   */
  [[nodiscard]] double getYSize() const { return sizeY_; }

  /**
   * Get the center position of the heightmap in x-axis
   * @return position
   */
  [[nodiscard]] double getCenterX() const { return centerX_; }

  /**
   * Get the center position of the heightmap in y-axis
   * @return position
   */
  [[nodiscard]] double getCenterY() const { return centerY_; }

  /**
   * Get the vector of height values
   * @return vector containing height values
   */
  [[nodiscard]] const std::vector<double> &getHeightVector() const { return height_; }

  /**
   * If the height is updated. This triggers the server to send the geomtry again
   */
  void setUpdatedFalse() { updated_ = false; }

  /**
   * Get if the height is updated
   * @return if the height is updated or not
   */
  [[nodiscard]] bool isUpdated() const { return updated_; }

  ~HeightMap() final;

  void updateCollision() final;

  void setPosition(const Eigen::Vector3d &originPosition) final {
    SingleBodyObject::setPosition(originPosition);
    updateCollision();
  }

  void setPosition(double x, double y, double z) final {
    SingleBodyObject::setPosition(x, y, z);
    updateCollision();
  }

  void setPosition(const Vec<3>& pos) final {
    SingleBodyObject::setPosition(pos);
    updateCollision();
  }

  void setOrientation(const Eigen::Quaterniond &quaternion) final {
    SingleBodyObject::setOrientation(quaternion);
    updateCollision();
  }

  void setOrientation(const Eigen::Vector4d &quaternion) final {
    SingleBodyObject::setOrientation(quaternion);
    updateCollision();
  }

  void setOrientation(double w, double x, double y, double z) final {
    SingleBodyObject::setOrientation(w, x, y, z);
    updateCollision();
  }

  void setOrientation(const Eigen::Matrix3d &rotationMatrix) final {
    SingleBodyObject::setOrientation(rotationMatrix);
    updateCollision();
  }

  /**
   * Set varying color on the heightmap
   * If set, the appearance will be ignored.
   * Set emtpy vector to the color level to make the appearance active
   * @param color1 first color
   * @param color2 second color
   * @param colorLevel color level
   */
  void setColor(const std::vector<ColorRGB>& colorLevel) {
    RSFATAL_IF(colorLevel.size() != height_.size(), "The color levels should be the same size as the vertices (heights).")
    updated_ = true;
    colorMap_ = colorLevel;
  }

  /**
   * Get the whole color vector.
   * If set, the appearance will be ignored in unreal. Unity color map is not supported yet
   * @return color
   */
  [[nodiscard]] const std::vector<ColorRGB>& getColorMap() const { return colorMap_; }

  /**
   * Returns color of the heightmap at specified point. The color from an RGB camera can be different from this because this is the base color and lighting changes the appearance.
   * @param x
   * @param y
   * @return the RGB color
   */
  [[nodiscard]] ColorRGB getColor(double x, double y) const {
    double normalizedX = (x - centerX_ + .5 * sizeX_) / sizeX_ * double(xSamples_-1);
    double normalizedY = (y - centerY_ + .5 * sizeY_) / sizeY_ * double(ySamples_-1);
    auto gridX = std::min(std::max(size_t(normalizedX), size_t(0)), xSamples_-1);
    auto gridY = std::min(std::max(size_t(normalizedY), size_t(0)), ySamples_-1);
    auto nextX = std::min(std::max(size_t(normalizedX) + size_t(1), size_t(0)), xSamples_-1);
    auto nextY = std::min(std::max(size_t(normalizedY) + size_t(1), size_t(0)), ySamples_-1);

    double xPercent = normalizedX - double(gridX);
    double yPercent = normalizedY - double(gridY);

    RSFATAL_IF(colorMap_.size() != height_.size(), "Color map is not specified")

    ColorRGB colorxy = colorMap_[gridY * xSamples_ + gridX];
    ColorRGB colorxy1 = colorMap_[nextY * xSamples_ + gridX];
    ColorRGB colorx1y = colorMap_[gridY * xSamples_ + nextX];
    ColorRGB colorx1y1 = colorMap_[nextY * xSamples_ + nextX];

    ColorRGB finalColor;
    finalColor.r = colorxy.r * (1. - xPercent) * (1. - yPercent) +
        colorxy1.r * (1. - xPercent) * yPercent +
        colorx1y.r * xPercent * (1. - yPercent) +
        colorx1y1.r * xPercent * yPercent;

    finalColor.g = colorxy.g * (1. - xPercent) * (1. - yPercent) +
        colorxy1.g * (1. - xPercent) * yPercent +
        colorx1y.g * xPercent * (1. - yPercent) +
        colorx1y1.g * xPercent * yPercent;

    finalColor.b = colorxy.b * (1. - xPercent) * (1. - yPercent) +
        colorxy1.b * (1. - xPercent) * yPercent +
        colorx1y.b * xPercent * (1. - yPercent) +
        colorx1y1.b * xPercent * yPercent;

    return finalColor;
  }

 private:
  void generateTerrain(const TerrainProperties &terrainProperties);
  std::vector<double> height_, odeHeight_;
  std::vector<ColorRGB> colorMap_;
  bool updated_ = false;
  dHeightfieldDataID heightFieldData_;
  double centerX_, centerY_, sizeX_, sizeY_;
  size_t xSamples_, ySamples_;
  Mat<3,3> rotOffset_;

};

}

#endif //RAISIM_HEIGHTMAP_HPP
