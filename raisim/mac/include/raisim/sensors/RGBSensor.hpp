//
// Created by jemin on 2022-07-20.
//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_RGBSENSOR_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_RGBSENSOR_HPP_


#include "raisim/sensors/Sensors.hpp"


namespace raisim {

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
    std::string format = "R8G8B8A8"; // format is currently fixed
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

  /**
   * rgb image in bgra format. It is updated only if the measurement source is the visualizer and raisimUnreal is used
   * @return The image data in char vector
   */
  [[nodiscard]] const std::vector<char>& getImageBuffer () const { return rgbBuffer_; }
  [[nodiscard]] std::vector<char>& getImageBuffer () { return rgbBuffer_; }

  void update (class World& world) final {
    RSFATAL("RGB sensors can only be updated by the visualizer. Use RaisimUnreal. ")
  }

 private:
  RGBCameraProperties prop_;
  std::vector<char> rgbBuffer_;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_RGBSENSOR_HPP_
