//
// Created by jemin on 2022-07-20.
//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_DEPTHSENSOR_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_DEPTHSENSOR_HPP_

#include "raisim/sensors/Sensors.hpp"

namespace raisim {

class DepthCamera final : public Sensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum Frame : unsigned {
    SENSOR_FRAME = 0,
    ROOT_FRAME,
    WORLD_FRAME
  };

  struct DepthCameraProperties {
    std::string name, full_name;

    int width, height;
    int xOffset = 0, yOffset = 0;
    double clipNear, clipFar;
    double hFOV;

    /// noise type - CURRENTLY NOT USED
    enum class NoiseType : int {
      GAUSSIAN = 0,
      UNIFORM,
      NO_NOISE
    } noiseType;

    static NoiseType stringToNoiseType(const std::string& type) {
      if(type == "gaussian" || type == "Gaussian")
        return NoiseType::GAUSSIAN;
      else if (type == "uniform" || type == "Uniform")
        return NoiseType::UNIFORM;
      else
        return NoiseType::NO_NOISE;
    }
    double mean = 0., std;
    std::string format = "16";
  };

  explicit DepthCamera(const DepthCameraProperties& prop, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      Sensor(prop.name, prop.full_name, Sensor::Type::DEPTH, as, pos, rot, MeasurementSource::VISUALIZER), prop_(prop) {
    updateRayDirections();
  }
  ~DepthCamera() final = default;

  /**
   * This method must be called after sensor properties are modified. It updates the ray directions
   */
  void updateRayDirections() {
    depthArray_.resize(prop_.height * prop_.width);
    threeDPoints_.resize(prop_.height * prop_.width);
    precomputedRayDir_.reserve(prop_.height * prop_.width);

    const double hRef = std::tan(prop_.hFOV * 0.5) * 2.;
    const double vRef = hRef * double(prop_.height) / double(prop_.width);

    for (int j = 0; j < prop_.height; j++) {
      for (int i= 0; i < prop_.width; i++) {
        Vec<3> dirB;
        dirB.e() << 1., -hRef * (double(i+prop_.xOffset)+0.5) / double(prop_.width) + hRef * 0.5,
            -vRef * (double(j+prop_.yOffset)+0.5) / double(prop_.height) + vRef * 0.5;
        precomputedRayDir_.push_back(dirB);
      }
    }
  }

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.full_name, prop_.width, prop_.height, prop_.clipNear, prop_.clipFar,
                       prop_.hFOV, prop_.noiseType, prop_.mean, prop_.std, prop_.format);
  }

  [[nodiscard]] static Type getType() { return Type::DEPTH; }

  /**
   * This method is only useful on the real robot (and you use raisim on the real robot).
   * You can set the 3d point array manually
   * @param[in] data The 3d point data */
  void set3DPoints (const std::vector<raisim::Vec<3>>& data) {
    for (int i = 0; i < prop_.width * prop_.height; i++)
      threeDPoints_[i] = data[i];
  }

  /** This method will return garbage if it has never been updated. Makes sure that the updateTimeStep is positive (negative means that it has never been updated)
   * @return depthArray
   */
  [[nodiscard]] const std::vector<float> & getDepthArray () const { return depthArray_; }
  [[nodiscard]] std::vector<float> & getDepthArray () { return depthArray_; }

  /**
   * Set the data manually. This can be useful on the real robot
   * @param[in] depthIn Depth values
   * @return The image data in char vector */
  void setDepthArray(const std::vector<float> & depthIn) {
    RSFATAL_IF(depthIn.size() != depthArray_.size(), "Input data size should be "<<prop_.width <<" by "<<prop_.height);
    depthArray_ = depthIn;
  }

  /** This method works only if the measurement source is Raisim
   * @return 3D points in the world frame */
  [[nodiscard]] const std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>>& get3DPoints() const { return threeDPoints_; }

  /**
   * Get the depth camera properties. MUST call updateRayDirections() after modifying the sensor properties.
   * @return the depth camera properties */
  [[nodiscard]] DepthCameraProperties& getProperties () { return prop_; }

  /**
   * Update the sensor value
   * @param[in] world the world that the sensor is in
   */
  void update (class World& world) final;

  /**
   * Convert the depth values to 3D coordinates
   * @param[in] depthArray input depth array to convert
   * @param[out] pointCloud output point cloud
   * @param[in] sensorFrame If the 3D points are expressed in the sensor frame or the world frame */
  void depthToPointCloud(const std::vector<float>& depthArray, std::vector<raisim::Vec<3>>& pointCloud, bool isInSensorFrame = false) const;

 private:
  DepthCameraProperties prop_;
  std::vector<float> depthArray_;
  std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>> threeDPoints_;
  std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>> precomputedRayDir_;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_DEPTHSENSOR_HPP_
