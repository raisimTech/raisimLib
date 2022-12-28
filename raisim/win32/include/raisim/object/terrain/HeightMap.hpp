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
   * final color is color1*(1-colorLevel) + color2*colorLevel
   * If set, the appearance will be ignored.
   * Set emtpy vector to the color level to make the appearance active
   * @param color1 first color
   * @param color2 second color
   * @param colorLevel color level
   */
  void setColor(const Vec<3>& color1, const Vec<3>& color2, const std::vector<float>& colorLevel) {
    RSFATAL_IF(colorLevel.size() != height_.size(), "The color levels should be the same size as the vertices (heights).")
    updated_ = true;
    color1_ = color1;
    color2_ = color2;
    colorLevel_ = colorLevel;
  }

  /**
   * Get the first color.
   * final color is color1*(1-colorLevel) + color2*colorLevel
   * If set, the appearance will be ignored.
   * @return color1
   */
  [[nodiscard]] const Vec<3>& getColor1() const { return color1_; }

  /**
   * Get the second color.
   * Final color is color1*(1-colorLevel) + color2*colorLevel.
   * If set, the appearance will be ignored.
   * @return color2
   */
  [[nodiscard]] const Vec<3>& getColor2() const { return color2_; }

  /**
   * Get the color level.
   * Final color is color1*(1-colorLevel) + color2*colorLevel.
   * If set, the appearance will be ignored.
   * @return color level
   */
  [[nodiscard]] const std::vector<float>& getColorLevel() const { return colorLevel_; }

 private:
  void generateTerrain(const TerrainProperties &terrainProperties);
  std::vector<double> height_, odeHeight_;
  std::vector<float> colorLevel_; /// visualization color. final color is color1*(1-colorLevel) + color2*colorLevel
  Vec<3> color1_, color2_;
  bool updated_ = false;
  dHeightfieldDataID heightFieldData_;
  double centerX_, centerY_, sizeX_, sizeY_;
  size_t xSamples_, ySamples_;
  Mat<3,3> rotOffset_;

};

}

#endif //RAISIM_HEIGHTMAP_HPP
