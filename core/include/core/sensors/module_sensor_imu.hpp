// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_SENSORS_MODULE_SENSOR_IMU_HPP_
#define CORE_INCLUDE_CORE_SENSORS_MODULE_SENSOR_IMU_HPP_

#include <string>

#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"

namespace core::sensors {
/**
 * @brief a class module sensor used to interface the Imu sensor
 *
 */
class ImuSensorModule : public SensorModuleAbs<utils::ImuData> {
 public:
  /**
   * @brief Construct a new Imu Sensor Module object
   *
   * @param name sensor module's name
   * @param debug enable/disable debug, default is false
   */
  explicit ImuSensorModule(const std::string& name, const bool debug = false);

  /**
   * @brief Destroy the Imu Sensor Module object
   *
   */
  ~ImuSensorModule() = default;

  void Initialize() override;
  bool Probe() override;
  bool Test() override;
  void Update() override;

 protected:
 private:
};

}  // namespace core::sensors
#endif  // CORE_INCLUDE_CORE_SENSORS_MODULE_SENSOR_IMU_HPP_
