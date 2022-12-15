// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/sensors/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace hardware::common::sensors {
int16_t To16Bit(uint8_t msb, uint8_t lsb) {
  const auto high = static_cast<int>(msb);
  const auto low = static_cast<int>(lsb);
  constexpr auto bit_shift = 8;
  return static_cast<int16_t>(high << bit_shift | low);
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
  printf("current 0x%x, mask 0x%x, flag 0x%x, result 0x%x\n", byte, mask, flag,
         static_cast<uint8_t>(updated));
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

}  // namespace hardware::common::sensors
