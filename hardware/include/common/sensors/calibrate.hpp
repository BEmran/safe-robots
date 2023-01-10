// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_SENSORS_CALIBRATE_HPP_
#define HARDWARE_COMMON_SENSORS_CALIBRATE_HPP_

#include <functional>
#include <optional>

#include "common/sensors/utils.hpp"
#include "core/math/math.hpp"

namespace hardware::common::sensors {
using ReadFunc = std::function<core::math::Vec3(void)>;
// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.

std::optional<SensorSpecs<3>>
CalibrateAccelerometer(const ReadFunc& cb, const SensorSpecs<3>& spec);

std::optional<SensorSpecs<3>> CalibrateGyroscope(const ReadFunc& cb,
                                                 const SensorSpecs<3>& spec);

std::optional<SensorSpecs<3>> CalibrateMagnetometer(const ReadFunc& cb,
                                                    const SensorSpecs<3>& spec);

}  // namespace hardware::common::sensors
#endif  // HARDWARE_COMMON_SENSORS_CALIBRATE_HPP_
