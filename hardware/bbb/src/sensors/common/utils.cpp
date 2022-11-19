// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "sensors/common/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace sensors::common::utils {
int16_t ToWord(const Bytes bytes) {
  constexpr auto bit_shift = 8;
  const int high = static_cast<int>(bytes.msb) << bit_shift;
  const int low = static_cast<int>(bytes.lsb);
  return static_cast<int16_t>(high | low);
}

Bytes ToBytes(const int16_t word) {
  constexpr auto bit_shift = 8;
  constexpr auto mask = 0xFF;
  const uint8_t msb = static_cast<uint8_t>((word >> bit_shift) & mask);
  const uint8_t lsb = static_cast<uint8_t>(word & mask);
  return Bytes{msb, lsb};
}

Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array) {
  return Vec3{array[0], array[1], array[2]};
}

Vec3 Vec3From16BitsVector(std::vector<int16_t>::const_iterator begin) {
  Vec3 vec;
  for (auto i = 0; i < vec.size(); i++) {
    vec[i] = static_cast<float>(*(begin + i));
  }
  return vec;
}

uint8_t SetFlags(uint8_t byte, uint8_t mask, uint8_t flag) {
  const auto updated = (byte & mask) | flag;
  return static_cast<uint8_t>(updated);
}

Vec3 EstimateRPY(const Vec3& accel) {
  const auto accel_normalized = accel.normalized();
  const auto ay2 = accel_normalized.y() * accel_normalized.y();
  const auto az2 = accel_normalized.z() * accel_normalized.z();

  const auto rx =
    std::atan2(accel_normalized.y(), accel_normalized.z()) / cu::DEG_TO_RAD;
  const auto ry =
    std::atan2(-accel_normalized.x(), std::sqrt(ay2 + az2)) / cu::DEG_TO_RAD;
  const auto rz = 0.F;
  return Vec3(rx, ry, rz);
}

}  // namespace sensors::common::utils
