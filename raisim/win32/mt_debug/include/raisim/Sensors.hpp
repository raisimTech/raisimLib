//----------------------------//
// This file is part of RaiSim//
// Copyright 2021, RaiSim Tech//
//----------------------------//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_HPP_

#include <string>
#include <Eigen/Core>
#include "raisim/math.hpp"
#include "raisim/helper.hpp"
#include "raisim/server/SerializationHelper.hpp"


namespace raisim {

class Sensor {
 public:
  enum class Type : int {
    UNKNOWN = 0,
    RGB,
    DEPTH
  };

  enum class Source : int {
    RAISIM = 0, // raisim automatically updates the measurements according to the simulation time
    VISUALIZER, // visualizer automatically updates the measurements according to the simulation time
    MANUAL // user manually update the measurements whenever needed
  };

  Sensor (std::string name, Type type, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      name_(std::move(name)), type_(type), as_(as), posB_(pos), rotB_(rot) { }
  virtual ~Sensor() = default;
  void setPose(const Vec<3>& pos, const Mat<3,3>& rot) {
    pos_ = pos;
    rot_ = rot;
  }

  [[nodiscard]] const Vec<3>& getPos() { return pos_; }
  [[nodiscard]] const Mat<3,3>& getRot() { return rot_; }
  [[nodiscard]] const Vec<3>& getPosInSensorFrame() { return posB_; }
  [[nodiscard]] const Mat<3,3>& getRotInSensorFrame() { return rotB_; }
  void setPosInSensorFrame(const Vec<3>& pos) { posB_ = pos; }
  void setRotInSensorFrame(const Mat<3,3>& rot) { rotB_ = rot; }
  const std::string& getName() { return name_; }
  [[nodiscard]] Type getType() { return type_; }
  void setFrameId(size_t id) { frameId_ = id; }
  [[nodiscard]] double getUpdateRate() const { return updateRate_; }
  [[nodiscard]] double getUpdateTimeStamp() const { return updateTimeStamp_; }
  void setUpdateRate(double rate) { updateRate_ = rate; }
  void setUpdateTimeStamp(double time) { updateTimeStamp_ = time; }
  virtual char* serializeProp (char* data) const = 0;
  virtual void updatePose(class World &world) = 0;
  [[nodiscard]] Source getSource() { return source_; }
  void setSource(Source source) { source_ = source; }

 protected:
  Type type_;
  Vec<3> pos_, posB_;
  Mat<3,3> rot_, rotB_;
  size_t frameId_;
  class ArticulatedSystem* as_;
  Source source_ = Source::MANUAL;

 private:
  std::string name_;
  double updateRate_, updateTimeStamp_ = -1.;
};

static inline std::string toString(Sensor::Type type) {
  switch (type) {
    case Sensor::Type::DEPTH:
      return "depth";
    case Sensor::Type::RGB:
      return "rgb";
    default:
      return "unknown";
  }
  return "unknown";
}

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
    depthArray_.setZero(prop.height, prop.width);
    threeDPoints_.resize(prop.height * prop.width);
  }
  ~DepthCamera() final = default;

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.name, prop_.width, prop_.height, prop_.clipNear, prop_.clipFar,
                       prop_.hFOV, prop_.noiseType, prop_.mean, prop_.std, prop_.format);
  }

  /*
   * This method is only useful on the real robot (and you use raisim on the real robot).
   * You can set the depth array manually
   */
  void setDepthArray (const Eigen::MatrixXd& data) { depthArray_.e() = data; }

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
  [[nodiscard]] const raisim::MatDyn& getDepthArray () const { return depthArray_; }

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

  void updatePose(class World &world) final;
  void update (class World& world);

 protected:

 private:
  DepthCameraProperties prop_;
  raisim::MatDyn depthArray_;
  std::vector<raisim::Vec<3>, AlignedAllocator<raisim::Vec<3>, 32>> threeDPoints_;
};

class RGBCamera : public Sensor {
 public:
  struct RGBCameraProperties {
    std::string name;
    int width, height;
    double clipNear, clipFar;
    double hFOV;

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
    std::string format = "R8G8B8";
  };

  RGBCamera(RGBCameraProperties& prop, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      Sensor(prop.name, Type::RGB, as, pos, rot), prop_(prop) {
    rgbBuffer_.resize(prop.height * prop.width * 4);
  }

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.name, prop_.width, prop_.height, prop_.clipNear, prop_.clipFar,
                       prop_.hFOV, prop_.noiseType, prop_.mean, prop_.std, prop_.format);
  }

  [[nodiscard]] static Type getType() { return Type::RGB; }
  [[nodiscard]] const RGBCameraProperties& getProperties () const { return prop_; }

  [[nodiscard]] const std::vector<char>& getImageBuffer () const { return rgbBuffer_; }
  [[nodiscard]] std::vector<char>& getImageBuffer () { return rgbBuffer_; }

  void updatePose(class World &world) final;

 private:
  RGBCameraProperties prop_;
  std::vector<char> rgbBuffer_;
};

}
#endif //RAISIM_INCLUDE_RAISIM_SENSORS_HPP_
