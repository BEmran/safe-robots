// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_SENSORS_UTILS_HPP_
#define HARDWARE_COMMON_SENSORS_UTILS_HPP_

#include <unistd.h>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "core/math/math.hpp"
#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"

namespace hardware::common::sensors {
using core::math::CreateScalar;
using core::math::DEG_TO_RAD;
using core::math::GRAVITY;
using core::math::Mat3;
using core::math::MATH_TYPE;
using core::math::PI;
using core::math::Vec3;
using core::utils::ImuData;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;

/// @brief multiply to convert degrees to radians
// constexpr double DEG_TO_RAD = 0.0174532925199;
/// @brief multiply to convert radians to degrees
constexpr double RAD_TO_DEG = 57.295779513;
/// @brief multiply to convert radians to degrees
constexpr double MS2_TO_G = 0.10197162129;
/// @brief multiply to convert G to m/s^2, standard gravity definition
constexpr double G_TO_MS2 = 9.80665;

template <typename Key>
struct ConfigMap {
  using PairType = std::pair<const Key, std::string>;
  using UnderlingType = std::underlying_type_t<Key>;
  std::map<Key, std::string> map;

  ConfigMap(std::initializer_list<PairType> l) : map{l} {
  }

  std::optional<Key> FindKey(const UnderlingType key_value) const {
    const auto it =
      std::find_if(map.cbegin(), map.cend(), [key_value](const PairType p) {
        return key_value == static_cast<UnderlingType>(p.first);
      });
    if (it == map.end()) {
      std::cout << "failed to find key" << std::endl;
      return {};
    }
    return it->first;
  }

  std::string operator[](const Key& key) const {
    const auto it = map.find(key);
    if (it == map.end()) {
      return "Undefined";
    }
    return it->second;
  }

  std::string Name(const std::optional<Key>& key) const {
    if (not key.has_value()) {
      return "Undefined";
    }
    return this->operator[](key.value());
  }
};

/**
 * @brief Hold sensor measurement specifications used to convert sensor raw
 * reading to usable data (scalded and in iso unit)
 * @details apply the equation
 * Res = ((misalignment * (raw - bias) / sensitivity) + offset)
 * Res = A * raw + b;
 * A = (misalignment / sensitivity
 * b = (- misalignment * bias / sensitivity + offset)
 */
template <int Size>
struct SensorSpecs {
  MATH_TYPE sensitivity;  // the smallest absolute amount of change that can be
                          // detected by a measurement = max_bit_count / scale
  MATH_TYPE unit_conversion;  // convert a raw value into iso unit

  core::math::Vector<Size> bias;
  core::math::Vector<Size> offset;
  core::math::Matrix<Size, Size> misalignment;
  core::math::Matrix<Size, Size> A;
  core::math::Vector<Size> b;
  /**
   * @brief Construct a new Sensor Specs object
   *
   */
  SensorSpecs() : SensorSpecs(1.0, 1.0) {
  }

  /**
   * @brief Construct a new Sensor Specs object
   *
   * @param sen sensitivity value
   * @param unit unit conversion row -> iso unit
   */
  SensorSpecs(MATH_TYPE sen, MATH_TYPE unit)
    : sensitivity(sen), unit_conversion(unit) {
    misalignment.setIdentity();
    offset.setZero();
    bias.setZero();
    UpdateEquation();
  }

  /**
   * @brief apply specs on a raw measurement
   *
   * @param raw raw data
   * @return MATH_TYPE the post proceeded data
   */
  inline MATH_TYPE Apply(MATH_TYPE raw) const {
    return A(0) * raw + b[0];
  }

  inline void SetCalibration(core::math::InputMat m, core::math::InputMat bias_,
                             core::math::InputMat offset_) {
    misalignment << m;
    bias << bias_;
    offset << offset_;
    UpdateEquation();
  }

  inline void UpdateEquation() {
    A << misalignment / sensitivity;
    b << (offset - misalignment * bias / sensitivity);
  }

  /**
   * @brief apply specs on a vector raw measurement
   *
   * @param raw raw data vector
   * @return Vec3 the post proceeded data
   */
  inline Vec3 Apply(core::math::InputMat raw) const {
    return (A * raw + b) * unit_conversion;
  }
};

/**
 * @brief Turn the MSB and LSB into a signed 16-bit value
 *
 * @param msb most significant bits (High 8-bits)
 * @param lsb least significant bits (Low 8-bits)
 * @return int16_t full number
 */
int16_t To16Bit(uint8_t msb, uint8_t lsb);

/**
 * @brief Convert a std::array data to Vec3
 *
 * @param array std array of size 3
 * @return Vec3 vector of dim 3
 */
Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array);

Vec3 Vec3From16BitsVector(std::vector<int16_t>::const_iterator begin);

uint8_t SetFlags(uint8_t byte, uint8_t mask, uint8_t flag);

/**
 * @brief Estimate roll and pitch angles from accelerometer
 *
 * @param accel accelerometer sensor data
 * @return Vec3 angle vector [roll, pitch, yaw]
 */
Vec3 EstimateRPY(const Vec3& accel);

}  // namespace hardware::common::sensors

#endif  // HARDWARE_COMMON_SENSORS_UTILS_HPP_
