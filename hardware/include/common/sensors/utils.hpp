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

#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"
#include "core/utils/math.hpp"

namespace hardware::common::sensors {
using core::utils::DEG_TO_RAD;
using core::utils::GRAVITY;
using core::utils::ImuData;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::PI;
using core::utils::Vec3;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;
using core::utils::CreateScalar;

/// @brief multiply to convert degrees to radians
// constexpr double DEG_TO_RAD = 0.0174532925199;
/// @brief multiply to convert radians to degrees
constexpr double RAD_TO_DEG = 57.295779513;
/// @brief multiply to convert radians to degrees
constexpr double MS2_TO_G = 0.10197162129;
/// @brief multiply to convert G to m/s^2, standard gravity definition
constexpr double G_TO_MS2 = 9.80665;

struct SimpleSpecInfo {
  std::string name;
  SimpleSpecInfo() : name("") {
  }
  SimpleSpecInfo(std::string_view name_) : name(name_) {
  }
  std::string Name() const {
    return name;
  }
};

struct SpecInfo {
  float value;
  std::string name;
  SpecInfo() : value{0.F}, name("") {
  }
  SpecInfo(const float value_, std::string_view name_)
    : value(value_), name(name_) {
  }
  std::string Name() const {
    return name;
  }
};

template <typename Key, typename T>
struct ConfigMap {
  using PairType = std::pair<const Key, T>;
  using UnderlingType = std::underlying_type_t<Key>;
  std::map<Key, T> map;

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

  T& operator[](const Key& key) {
    return map[key];
  }

  std::string Name(const std::optional<Key>& key) {
    if (not key.has_value()) {
      return "Undefined";
    }
    return map[key.value()].Name();
  }
};

template <typename Key>
using SpecInfoMap = ConfigMap<Key, SpecInfo>;

template <typename Key>
using SimpleSpecInfoMap = ConfigMap<Key, SimpleSpecInfo>;

/**
 * @brief Hold sensor measurement specifications used to convert sensor raw
 * reading to usable data (scalded and in iso unit)
 * @details apply the equation
 * Res = ((misalignment * (raw - bias) / sensitivity) + offset) *
 * unit_conversion; Res = A * raw + b; A = (misalignment / sensitivity *
 * unit_conversion; b = (- misalignment * bias / sensitivity + offset) *
 * unit_conversion;
 */
template <int Size>
struct SensorSpecs {
  MATH_TYPE sensitivity;  // the smallest absolute amount of change that can be
                          // detected by a measurement = max_bit_count / scale
  MATH_TYPE unit_conversion;  // convert a raw value into iso unit

  core::utils::Vector<Size> bias;
  core::utils::Vector<Size> offset;
  core::utils::Matrix<Size, Size> misalignment;
  core::utils::Matrix<Size, Size> A;
  core::utils::Vector<Size> b;
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

  inline void SetCalibration(core::utils::InputMat m,
                             core::utils::InputMat bias_,
                             core::utils::InputMat offset_) {
    misalignment << m;
    bias << bias_;
    offset << offset_;
    UpdateEquation();
  }

  inline void UpdateEquation() {
    A << misalignment * unit_conversion / sensitivity;
    b << (offset - misalignment * bias / sensitivity) * unit_conversion;
  }

  /**
   * @brief apply specs on a vector raw measurement
   *
   * @param raw raw data vector
   * @return Vec3 the post proceeded data
   */
  inline Vec3 Apply(core::utils::InputMat raw) const {
    return A * raw + b;
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
