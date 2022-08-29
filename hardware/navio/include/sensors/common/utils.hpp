// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_COMMON_UTILS_HPP_
#define SENSORS_COMMON_UTILS_HPP_

#include <unistd.h>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "core/sensors/module_sensor.hpp"
#include "core/utils/data.hpp"
#include "core/utils/math.hpp"

namespace sensors::common::utils {
using core::utils::GRAVITY;
using core::utils::ImuData;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::PI;
using core::utils::RAD_TO_DEG;
using core::utils::Vec3;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;

namespace literals {
/**
 * @brief User litterer to define a number of type uint8_t
 *
 */
inline constexpr uint8_t operator"" _uc(
  unsigned long long arg) noexcept {  // NOLINT [runtime/int] TODO(Bara)
  return static_cast<uint8_t>(arg);
}
}  // namespace literals

using namespace literals;  // NOLINT [build/namespaces_literals] TODO(Bara)

struct SpecInfo {
  float value;
  std::string name;
  SpecInfo() : SpecInfo(0.F, "") {
  }
  SpecInfo(float value_, const std::string& name_)
    : value(value_), name(name_) {
  }
};

template <typename E>
class SpecInfoMap {
  using Map = std::map<E, SpecInfo>;
  using Iterator = typename Map::iterator;

 public:
  explicit SpecInfoMap(const Map& map) : map_(map) {
  }

  inline E FindByByte(uint8_t byte) {
    auto it = std::find_if(map_.begin(), map_.end(), [byte](auto ele) {
      return byte == static_cast<uint8_t>(ele.first);
    });
    if (it == map_.end()) {
      throw std::runtime_error("undefined specification");
    }
    return it->first;
  }

  inline SpecInfo operator[](E key) {
    return map_[key];
  }

  inline uint8_t Byte(E key) {
    return static_cast<uint8_t>(key);
  }

 private:
  Map map_;
};

/**
 * @brief Hold sensor measurement specifications used to convert sensor raw
 * reading to usable data (scalded and in iso unit)
 *
 */
struct SensorSpecs {
  MATH_TYPE sensitivity;  // the smallest absolute amount of change that can be
                          // detected by a measurement = max_bit_count / scale
  MATH_TYPE unit_conversion;  // convert a raw value into iso unit

  Vec3 bias = Vec3::Zero();
  Vec3 offset = Vec3::Zero();
  Mat3 misalignment = Mat3::Identity();
  Mat3 A = Mat3::Identity();
  Vec3 b = Vec3::Zero();
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
  }

  /**
   * @brief Construct a new Sensor Specs object
   *
   * @param sen sensitivity value
   * @param unit unit conversion row -> iso unit
   */
  SensorSpecs(MATH_TYPE sen, MATH_TYPE unit, const Vec3& bias_,
              const Vec3& offset_)
    : sensitivity(sen), unit_conversion(unit), bias(bias_), offset(offset_) {
    UpdateEquation();
  }

  /**
   * @brief apply specs on a raw measurement
   *
   * @param raw raw data
   * @return MATH_TYPE the post proceeded data
   */
  inline MATH_TYPE Apply(MATH_TYPE raw) const {
    // return (raw - bias[0]) * (unit_conversion / sensitivity) + offset[0];
    return A(0) * raw + b[0];
  }

  inline void SetCalibration(const Mat3& m, const Vec3& bias_,
                             const Vec3& offset_) {
    misalignment = m;
    bias = bias_;
    offset = offset_;
    UpdateEquation();
  }

  inline void UpdateEquation() {
    A = misalignment * unit_conversion / sensitivity;
    b = misalignment * (offset - bias / sensitivity) * unit_conversion;
  }

  /**
   * @brief apply specs on a vector raw measurement
   *
   * @param raw raw data vector
   * @return Vec3 the post proceeded data
   */
  inline Vec3 Apply(const Vec3& raw) const {
    // return ((misalignment * (raw - bias) / sensitivity) + offset) *
    // unit_conversion;
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

}  // namespace sensors::common::utils

#endif  // SENSORS_COMMON_UTILS_HPP_
