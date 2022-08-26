#include "mpu/my_utils.hpp"
#include <iostream>

#include <algorithm>
#include <cmath>

namespace mpu
{

void Delay(uint32_t msec)
{
  usleep(msec * 1000);
}

int16_t To16Bit(const uint8_t msb, const uint8_t lsb)
{
  const auto high = static_cast<int>(msb);
  const auto low = static_cast<int>(lsb);
  return static_cast<int16_t>(high << 8 | low);
}

SensorSpecs CreateSensorSpecs(const MATH_TYPE scale, const MATH_TYPE unit)
{
  const auto sen = max_bit_val / scale;
  std::cout << "scale: " << scale << ", sensitivity: " << sen
            << " unit:" << unit << std::endl;
  return SensorSpecs(sen, unit);
}

Vec3 ArrayToVec3(const std::array<MATH_TYPE, 3>& array)
{
  return Vec3{array[0], array[1], array[2]};
}

Vec3 Vec3From16Bits(const std::vector<int16_t>::const_iterator begin)
{
  Vec3 vec;
  for (auto i = 0; i < vec.size(); i++)
  {
    vec[i] = static_cast<float>(*(begin + i));
  }
  return vec;
}

Vec3 ApplySensorSpecs(const Vec3& raw, const SensorSpecs& spec)
{
  Vec3 vec;
  for (size_t i = 0; i < raw.size(); i++)
  {
    vec[i] = spec.Apply(static_cast<MATH_TYPE>(raw[i]));
  }
  return vec;
}

std::array<MATH_TYPE, 3> ApplySensorSpecs(const std::array<int16_t, 3>& raw,
                                          const SensorSpecs& spec)
{
  std::array<MATH_TYPE, 3> data;
  std::transform(raw.begin(), raw.end(), data.begin(), [&spec](const auto r) {
    return spec.Apply(static_cast<MATH_TYPE>(r));
  });
  return data;
}

void PrintVec(const std::vector<uint8_t>& vec)
{
  std::for_each(vec.begin(), vec.end(), [](const auto data) {
    std::cout << static_cast<int>(data) << "\t";
  });
  std::cout << std::endl;
}

Vec3 EstimateRPY(const Vec3& accel)
{
  const auto accel_normalized = accel.normalized();
  const auto ay2 = accel_normalized.y() * accel_normalized.y();
  const auto az2 = accel_normalized.z() * accel_normalized.z();

  const auto rx = std::atan2(accel_normalized.y(), accel_normalized.z());
  const auto ry = std::atan2(-accel_normalized.x(), std::sqrt(ay2 + az2));
  const auto rz = 0.F;
  return Vec3(rx, ry, rz);
}

}  // namespace mpu
