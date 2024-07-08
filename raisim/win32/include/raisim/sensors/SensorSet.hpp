//
// Created by jemin on 24. 6. 12.
//

#ifndef RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_
#define RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_

#include <memory>
#include <unordered_map>
#include <vector>
#include "Sensors.hpp"

namespace raisim {

namespace urdf {
  class LoadFromURDF2;
}

class SensorSet {
 public:
  friend class raisim::Sensor;
  friend class raisim::Child;
  friend class raisim::ArticulatedSystem;
  friend class raisim::urdf::LoadFromURDF2;
//  using SensorSetDataType = std::unordered_map<std::string, std::shared_ptr<Sensor>, std::hash<std::string>, std::equal_to<>, AlignedAllocator<std::pair<const std::string, std::shared_ptr<Sensor>>,32>>;
  using SensorSetDataType = std::vector<Sensor*>;

  SensorSet(std::string nameI, std::string modelI, std::string serialNumberI);
  ~SensorSet();

  [[nodiscard]] SensorSetDataType& getSensors() {
    return sensors;
  }

  [[nodiscard]] const SensorSetDataType& getSensors() const {
    return sensors;
  }

  /**
 * @return sensor of the specified type
 */
  template<class T>
  T* getSensor(const std::string& nameIn) {
    auto sensorIter = std::find_if(sensors.begin(), sensors.end(), [&](Sensor* a){ return a->getName() == nameIn;});
    RSFATAL_IF(sensorIter == sensors.end(), "Cannot find \""<<nameIn<<"\"")
    auto sensor = *sensorIter;
    RSFATAL_IF(sensor->getType() != T::getType(), "Type mismatch. "
        << nameIn << " has a type of " << toString(sensor->getType()) << " and the requested type is "
        << toString(T::getType()))
    return reinterpret_cast<T*>(sensor);
  }

  /**
   * This function returns the sensor as its parent class pointer.
   * @param nameIn the name of the sensor
   * @return the sensor as a base class (Sensor*) pointer
   */
  Sensor* getSensorRawPtr(const std::string& nameIn) {
    auto sensorIter = std::find_if(sensors.begin(), sensors.end(), [&](Sensor* a){ return a->getName() == nameIn;});
    RSFATAL_IF(sensorIter == sensors.end(), "Cannot find \""<<nameIn<<"\"")
    auto sensor = *sensorIter;
    return sensor;
  }

  const std::string& getModel() { return model; }
  const std::string& getSerialNumber() { return serialNumber; };
  const std::string& getName() { return name; };

 protected:

  void addSensor (Sensor* sensor);

  std::string name;
  std::string serialNumber;
  std::string model;
  SensorSetDataType sensors;
};

}

#endif //RAISIM_INCLUDE_RAISIM_SENSORS_SENSORSET_HPP_
