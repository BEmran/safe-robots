// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_COMMON_CALIBRATE_HPP_
#define SENSORS_COMMON_CALIBRATE_HPP_

#include <functional>

#include "core/utils/math.hpp"
#include "sensors/common/utils.hpp"

namespace sensors::common::calibrate {
using ReadFunc = std::function<utils::Vec3(void)>;
using utils::SensorSpecs;
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.

SensorSpecs CalibrateAccelerometer(const ReadFunc& cb, const SensorSpecs& spec);

SensorSpecs CalibrateGyroscope(const ReadFunc& cb, const SensorSpecs& spec);

SensorSpecs CalibrateMagnetometer(const ReadFunc& cb, const SensorSpecs& spec);

}  // namespace sensors::common::calibrate
#endif  // SENSORS_COMMON_CALIBRATE_HPP_
