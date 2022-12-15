// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef HARDWARE_COMMON_SENSORS_MPU_DEF_HPP_
#define HARDWARE_COMMON_SENSORS_MPU_DEF_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <utility>

#include "common/sensors/mpu/mpu9250_register_map.hpp"
#include "common/sensors/utils.hpp"

namespace hardware::common::sensors::mpu {

constexpr int kMaxBitVal = 32760;  // Measurement range is from -32760 ~ +32760
                                   // decimal in 16-bit output.
constexpr int kMaxUTesla = 4912;   // Magnetic flux density in micro Tesla
constexpr auto TempSensitivity = 333.87F;
const auto kTempBias = 21.F;
const auto kTempOffset = 21.F;

// using namespace common::utils::literals;  // NOLINT
// [build/namespaces_literals] TODO(Bara)

enum class GyroScale : uint8_t {
  FS_250DPS = 0x00,
  FS_500DPS = 0x08,
  FS_1000DPS = 0x10,
  FS_2000DPS = 0x18
};

enum class GyroBW : uint8_t {
  BW_250HZ = 0x00,
  BW_184HZ = 0x01,
  BW_92HZ = 0x02,
  BW_41HZ = 0x03,
  BW_20HZ = 0x04,
  BW_10HZ = 0x05,
  BW_5HZ = 0x06,
  BW_3600HZ = 0x07
};

enum class AccelScale : uint8_t {
  FS_2G = 0x00,
  FS_4G = 0x08,
  FS_8G = 0x10,
  FS_16G = 0x18
};

enum class AccelBW : uint8_t {
  BW_218HZ = 0x01,
  BW_99HZ = 0x02,
  BW_44HZ = 0x03,
  BW_21HZ = 0x04,
  BW_10HZ = 0x05,
  BW_5HZ = 0x06
};

enum class MagScale : uint8_t { FS_14BITS = 0x00, FS_16BITS = 0x10 };

enum class MagMode : uint8_t {
  POWER_DOWN = 0x00,
  SINGLE_MEASURE = 0x01,
  CONTINUES_8HZ = 0x02,
  EXTERNAL_TRIGGER = 0x04,
  CONTINUES_100HZ = 0x06,
  SELF_TEST = 0x08,
  FUSE_ROM_ACCESS = 0x0F
};

inline auto AccelBWMap() {
  const static SimpleSpecInfoMap<AccelBW> map{
    {AccelBW::BW_218HZ, {"218 HZ"}},  //
    {AccelBW::BW_99HZ, {"99 HZ"}},    //
    {AccelBW::BW_44HZ, {"44 HZ"}},    //
    {AccelBW::BW_21HZ, {"21 HZ"}},    //
    {AccelBW::BW_10HZ, {"10 HZ"}},    //
    {AccelBW::BW_5HZ, {"5 HZ"}}};
  return map;
}

inline auto AccelScaleMap() {
  const static SpecInfoMap<AccelScale> map{{AccelScale::FS_2G, {2.F, "2G"}},
                                           {AccelScale::FS_4G, {4.F, "4G"}},
                                           {AccelScale::FS_8G, {8.F, "8G"}},
                                           {AccelScale::FS_16G, {16.F, "16G"}}};
  return map;
}

inline auto GyroBWMap() {
  const static SimpleSpecInfoMap<GyroBW> map{{GyroBW::BW_250HZ, {"250 HZ"}},  //
                                             {GyroBW::BW_184HZ, {"184 HZ"}},  //
                                             {GyroBW::BW_92HZ, {"92 HZ"}},    //
                                             {GyroBW::BW_41HZ, {"41 HZ"}},    //
                                             {GyroBW::BW_20HZ, {"20 HZ"}},    //
                                             {GyroBW::BW_10HZ, {"10 HZ"}},    //
                                             {GyroBW::BW_5HZ, {"5 HZ"}},      //
                                             {GyroBW::BW_3600HZ, {"3600 HZ"}}};
  return map;
}

inline auto GyroScaleMap() {
  const static SpecInfoMap<GyroScale> map{
    {GyroScale::FS_250DPS, {250.F, "250 DPS"}},
    {GyroScale::FS_500DPS, {500.F, "500 DPS"}},
    {GyroScale::FS_1000DPS, {1000.F, "1000 DPS"}},
    {GyroScale::FS_2000DPS, {2000.F, "2000 DPS"}}};
  return map;
}

inline auto MagScaleMap() {
  const static SpecInfoMap<MagScale> map{
    {MagScale::FS_14BITS, {0.25F * kMaxUTesla, "14 BITS"}},
    {MagScale::FS_16BITS, {1.00F * kMaxUTesla, "16 BITS"}}};
  return map;
}

inline auto MagModeMap() {
  const static SimpleSpecInfoMap<MagMode> map{
    {MagMode::POWER_DOWN, {"POWER DOWN"}},
    {MagMode::SINGLE_MEASURE, {"SINGLE MEASUREMENT"}},
    {MagMode::CONTINUES_8HZ, {"CONTINUES 8HZ"}},
    {MagMode::EXTERNAL_TRIGGER, {"EXTERNAL TRIGGER"}},
    {MagMode::CONTINUES_100HZ, {"CONTINUES 100HZ"}},
    {MagMode::SELF_TEST, {"SELF TEST"}},
    {MagMode::FUSE_ROM_ACCESS, {"FUSE ROM ACCESS"}}};
  return map;
}

struct SensorRawData {
  Vec3 accel = Vec3::Zero();
  Vec3 gyro = Vec3::Zero();
  Vec3 mag = Vec3::Zero();
  MATH_TYPE temp = 0;
  bool mag_over_flow = false;
};

constexpr AccelScale DefaultAccelScale{AccelScale::FS_16G};
constexpr AccelBW DefaultAccelBW{AccelBW::BW_44HZ};
constexpr GyroScale DefaultGyroScale{GyroScale::FS_2000DPS};
constexpr GyroBW DefaultGyroBW{GyroBW::BW_41HZ};
constexpr MagScale DefaultMagScale{MagScale::FS_16BITS};
constexpr MagMode DefaultMagMode{MagMode::CONTINUES_100HZ};
constexpr uint8_t DefaultSampleRateDivisor{4};

struct Config {
  std::optional<AccelScale> accel_scale{DefaultAccelScale};
  std::optional<AccelBW> accel_bw{DefaultAccelBW};
  std::optional<GyroScale> gyro_scale{DefaultGyroScale};
  std::optional<GyroBW> gyro_bw{DefaultGyroBW};
  std::optional<MagScale> mag_scale{DefaultMagScale};
  std::optional<MagMode> mag_mode{DefaultMagMode};
  uint8_t sample_rate_divisor{DefaultSampleRateDivisor};
};

}  // namespace hardware::common::sensors::mpu
#endif  // HARDWARE_COMMON_SENSORS_MPU_DEF_HPP_
