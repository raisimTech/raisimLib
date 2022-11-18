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
    std::string name;

    int width, height;
    double clipNear, clipFar;
    double hFOV;

    /// data type
    enum class DataType : unsigned {
      UNKNOWN = 0,
      COORDINATE,
      DEPTH_ARRAY,
    } dataType = DataType::DEPTH_ARRAY;

    static DataType stringToDataType(const std::string& type) {
      if(type == "coordinates" || type == "Coordinates")
        return DataType::COORDINATE;
      else if (type == "depth" || type == "Depth")
        return DataType::DEPTH_ARRAY;
      else return DataType::UNKNOWN;
    }

    /// noise type
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
      Sensor(prop.name, Sensor::Type::DEPTH, as, pos, rot), prop_(prop) {
    depthArray_.resize(prop.height * prop.width);
    threeDPoints_.resize(prop.height * prop.width);
  }
  ~DepthCamera() final = default;

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.name, prop_.width, prop_.height, prop_.clipNear, prop_.clipFar,
                       prop_.hFOV, prop_.noiseType, prop_.mean, prop_.std, prop_.format);
  }

  /*
   * This method is only useful on the real robot (and you use raisim on the real robot).
   * You can set the 3d point array manually
   */
  void set3DPoints (const std::vector<raisim::Vec<3>>& data) {
    for (int i = 0; i < prop_.width * prop_.height; i++)
      threeDPoints_[i] = data[i];
  }

  /* This method works only if the sensor update type is DEPTH_ARRAY.
   * Otherwise, it will return garbage.
   * In simulation, you can set the update type in the urdf file.
   * On the real robot, you can set the 3d data using setDepthArray method
   * @return depthArray
   */
  [[nodiscard]] const std::vector<float> & getDepthArray () const { return depthArray_; }
  [[nodiscard]] std::vector<float> & getDepthArray () { return depthArray_; }

  /* This method works only if the sensor update type is THREE_DIM_COORD.
   * Otherwise, it will return garbage.
   * In simulation, you can set the update type in the urdf file.
   * On the real robot, you can set the 3d data using set3DPoints method
   * @return 3D points in the specified frame. In simulation, the frame is specified in the URDF. On the real robot, you have to update the 3D points (using set3DPoints) in the correct frame.
   */
  [[nodiscard]] const std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>>& get3DPoints() const { return threeDPoints_; }

  [[nodiscard]] const DepthCameraProperties& getProperties () const { return prop_; }

  void setDataType(DepthCameraProperties::DataType type) { prop_.dataType = type; }

  [[nodiscard]] static Type getType() { return Type::DEPTH; }

  /*
   * Manually update the sensor pose then measurements using Raisim ray cast function.
   */
  void update (class World& world) final;

 protected:

 private:
  DepthCameraProperties prop_;
  std::vector<float> depthArray_;
  std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>> threeDPoints_;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_DEPTHSENSOR_HPP_
