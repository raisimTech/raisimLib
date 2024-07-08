//
// Created by jemin on 2024-04-25.
//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_SPINNINGLIDAR_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_SPINNINGLIDAR_HPP_

#include "raisim/sensors/Sensors.hpp"


namespace raisim {

class SpinningLidar : public Sensor {
 public:
  struct SpinningLidarProperties {
    std::string name, full_name;
    int yawSamples = 1, pitchSamples = 1;
    double pitchMinAngle = 0, pitchMaxAngle = 0;
    double rangeMin = 0, rangeMax = 100.0;
    double spinningRate = 10; // revolutions per second
    int spinDirection = 1; // positive for counter-clockwise, negative for clockwise
  };

  SpinningLidar (SpinningLidarProperties& prop, class ArticulatedSystem* as, const Vec<3>& pos, const Mat<3,3>& rot) :
      Sensor(prop.name, prop.full_name, Type::SPINNING_LIDAR, as, pos, rot, MeasurementSource::RAISIM), prop_(prop) {
    scan_.reserve(prop.pitchSamples * prop.yawSamples);
    timeStamp_ = 0;
  }

  char* serializeProp (char* data) const final {
    return server::set(data, type_, prop_.full_name, prop_.yawSamples, prop_.pitchSamples,
                       prop_.pitchMinAngle, prop_.pitchMaxAngle, prop_.rangeMin, prop_.rangeMax, prop_.spinningRate);
  }

  [[nodiscard]] static Type getType() { return Type::SPINNING_LIDAR; }
  [[nodiscard]] SpinningLidarProperties& getProperties () { return prop_; }

  void setProperties(const SpinningLidarProperties& prop) { 
    prop_ = prop;
    scan_.reserve(prop.pitchSamples * prop.yawSamples);
    timeStamp_ = 0;
  }

  [[nodiscard]] const std::vector<raisim::Vec<3>>& getScan() const { return scan_; }
  void setScan(const std::vector<raisim::Vec<3>>& scan) { scan_ = scan; }

  [[nodiscard]] double getTimeStamp() const { return timeStamp_; }
  void setTimeStamp(double timeStamp) { timeStamp_ = timeStamp; }

  [[nodiscard]] double getCurrentYaw() const { return currentYaw_; }

  /**
   * update lidar measurement
   */
  void update (class World& world) final;

 private:
  double timeStamp_;
  std::vector<raisim::Vec<3>> scan_;
  SpinningLidarProperties prop_;

  double currentYaw_ = 0;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_RGBSENSOR_HPP_
