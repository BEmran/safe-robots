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
using core::utils::DEG_TO_RAD;
using core::utils::GRAVITY;
using core::utils::ImuData;
using core::utils::Mat3;
using core::utils::MATH_TYPE;
using core::utils::PI;
using core::utils::Vec3;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;
using core::utils::CreateScalar;

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

template <typename T, typename N>
inline T Find(const std::map<T, N>& map, uint8_t byte) {
  const auto key = static_cast<T>(byte);
  auto it = map.find(key);
  if (it == map.end()) {
    throw std::runtime_error("undefined specification");
  }
  return it->first;
}

template <typename T>
inline auto ToByte(T t) {
  return static_cast<uint8_t>(t);
}

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
 * @brief simple struct hold the MSB and LSB of a word
 *
 */
struct Bytes {
  /// @brief most significant bits (High 8-bits)
  uint8_t msb{0};
  /// @brief least significant bits (Low 8-bits)
  uint8_t lsb{0};
  Bytes(const uint8_t msb_, const uint8_t lsb_) : msb{msb_}, lsb{lsb_} {
  }
};

/**
 * @brief Turn two bytes into a unsigned 16-bit value
 *
 * @param bytes 2 byte {msb, lsb}
 * @return int16_t full number
 */
int16_t ToWord(const Bytes bytes);

/**
 * @brief Turn a int16_t into two unsigned 8-bit value
 *
 * @param word full number
 * @return Bytes decoded two bytes
 */
Bytes ToBytes(const int16_t word);

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
