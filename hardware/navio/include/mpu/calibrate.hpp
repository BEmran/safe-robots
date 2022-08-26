#ifndef MPU_CALIBRATE_HPP
#define MPU_CALIBRATE_HPP

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>

#include <functional>

#include "mpu/my_utils.hpp"

namespace mpu
{

// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.

SensorSpecs CalibrateAccelerometer(const std::function<Vec3(void)>& cb,
                                   const SensorSpecs& spec);
SensorSpecs CalibrateGyroscope(const std::function<Vec3(void)>& cb,
                               const SensorSpecs& spec);
SensorSpecs CalibrateMagnetometer(const std::function<Vec3(void)>& cb,
                                  const SensorSpecs& spec);
// void CalibrateMagnetometer(std::shared_ptr<ImuSensorModule> sensor);

}  // namespace mpu
#endif  // MPU_CALIBRATE_HPP