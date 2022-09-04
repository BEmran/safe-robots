// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_LSM_LSM9DS1_REGISTER_MAP_HPP_
#define SENSORS_LSM_LSM9DS1_REGISTER_MAP_HPP_
#include <cstdint>
#include <map>
#include <string>

#include "sensors/common/utils.hpp"

namespace sensors::lsm {
namespace cu = common::utils;

constexpr int kMaxBitVal = 32767;  // Measurement range is from -32767 ~
                                   // +32767 decimal in 16-bit output.
constexpr auto kMagUnit = 100.0F;  // Magnetic flux density in uTesla

// TODO(Bara): change to single value instead of vector
const auto kTempSensitivity = 16.F;
const auto kTempBias = 0.F;
const auto kTempOffset = 25.F;

using namespace common::utils::literals;  // NOLINT
                                          // [build/namespaces_literals]
                                          // TODO(Bara)

// accel_scale defines all possible FSR's of the accelerometer:
enum class AccelScale : uint8_t {
  SCALE_2G = 0x00_uc,
  SCALE_16G = 0x08_uc,
  SCALE_4G = 0x10_uc,
  SCALE_8G = 0x18_uc
};

// accel_oder defines all possible output data rates of the accelerometer:
enum class AccelODR : uint8_t {
  POWER_DOWN = 0x00_uc,
  ODR_10HZ = 0x20_uc,
  ODR_50HZ = 0x40_uc,
  ODR_119HZ = 0x60_uc,
  ODR_238HZ = 0x80_uc,
  ODR_476HZ = 0xA0_uc,
  ODR_952HZ = 0xC0_uc,
};

// accel_abw defines all possible anti-aliasing filter rates of the
// accelerometer:
enum class AccelAntiAliasingBW : uint8_t {
  AA_BW_408 = 0x000_uc,
  AA_BW_211 = 0x010_uc,
  AA_BW_105 = 0x020_uc,
  AA_BW_50 = 0x030_uc,
};

// accel_abw defines all possible anti-aliasing filter rates of the
// accelerometer:
enum class AccelHighResolutionBW : uint8_t {
  HR_BW_DIV_50 = 0x000_uc,
  HR_BW_DIV_9 = 0x001_uc,
  HR_BW_DIV_100 = 0x002_uc,
  HR_BW_DIV_400 = 0x003_uc,
};
// gyro_scale defines the possible full-scale ranges of the gyroscope:
enum class GyroScale : uint8_t {
  SCALE_245DPS = 0x00_uc,
  SCALE_500DPS = 0x08_uc,
  SCALE_2000DPS = 0x18_uc
};

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
enum class GyroODR : uint8_t {
  ODR_14900mHZ = 0x20_uc,
  ODR_59500mHZ = 0x40_uc,
  ODR_119HZ = 0x60_uc,
  ODR_238HZ = 0x80_uc,
  ODR_476HZ = 0xA0_uc,
  ODR_952HZ = 0xC0_uc
};

// mag_odr defines all possible output data rates of the magnetometer:
enum class MagODR : uint8_t {
  ODR_0_625 = 0x00_uc,
  ODR_1_25 = 0x04_uc,
  ODR_2_50 = 0x08_uc,
  ODR_5 = 0x0C_uc,
  ODR_10 = 0x10_uc,
  ODR_20 = 0x14_uc,
  ODR_40 = 0x18_uc,
  ODR_80 = 0x1C_uc
};

enum class MagTempCompensation : uint8_t {
  DISABLED = 0x00_uc,
  ENABLED = 0x80_uc
};

enum class MagXYMode : uint8_t {
  LOW = 0x00_uc,
  MEDIUM = 0x20_uc,
  HIGH = 0x40_uc,
  ULTRA_HIGH = 0x60_uc
};

enum class MagZMode : uint8_t {
  LOW = 0x00_uc,
  MEDIUM = 0x04_uc,
  HIGH = 0x08_uc,
  ULTRA_HIGH = 0x0C_uc
};

enum class MagOperatingMode : uint8_t {
  CONTINUOUS = 0x00_uc,
  SINGLE = 0x01_uc,
  POWER_DOWN = 0x02_uc
};

// mag_scale defines all possible FSR's of the magnetometer:
enum class MagScale : uint8_t {
  SCALE_4GS = 0x00_uc,
  SCALE_8GS = 0x20_uc,
  SCALE_12GS = 0x40_uc,
  SCALE_16GS = 0x60_uc
};

inline auto AccelODRMap() {
  static std::map<AccelODR, std::string> map(
    {{AccelODR::POWER_DOWN, "POWER_DOWN"},
     {AccelODR::ODR_10HZ, "10 HZ"},
     {AccelODR::ODR_50HZ, "50 HZ"},
     {AccelODR::ODR_119HZ, "119 HZ"},
     {AccelODR::ODR_238HZ, "238 HZ"},
     {AccelODR::ODR_476HZ, "476 HZ"},
     {AccelODR::ODR_952HZ, "952 HZ"}});
  return map;
}

inline auto AccelScaleMap() {
  static std::map<AccelScale, cu::SpecInfo> map(
    {{AccelScale::SCALE_2G, {2.F, "2G"}},
     {AccelScale::SCALE_4G, {4.F, "4G"}},
     {AccelScale::SCALE_8G, {8.F, "8G"}},
     {AccelScale::SCALE_16G, {24.F, "16G"}}});  // DON"T CHANGE 0.000732
  return map;
}

inline auto AccelAntiAliasMap() {
  static std::map<AccelAntiAliasingBW, std::string> map(
    {{AccelAntiAliasingBW::AA_BW_408, "408 BW"},
     {AccelAntiAliasingBW::AA_BW_211, "211 BW"},
     {AccelAntiAliasingBW::AA_BW_105, "105 BW"},
     {AccelAntiAliasingBW::AA_BW_50, "50 BW"}});
  return map;
}

inline auto AccelHighResBWMap() {
  static std::map<AccelHighResolutionBW, std::string> map(
    {{AccelHighResolutionBW::HR_BW_DIV_50, "ODR/50"},
     {AccelHighResolutionBW::HR_BW_DIV_9, "ODR/9"},
     {AccelHighResolutionBW::HR_BW_DIV_100, "ODR/100"},
     {AccelHighResolutionBW::HR_BW_DIV_400, "ODR/400"}});
  return map;
}

inline auto GyroODRMap() {
  static std::map<GyroODR, std::string> map({{GyroODR::ODR_14900mHZ, "14.9 HZ"},
                                             {GyroODR::ODR_59500mHZ, "59.5 HZ"},
                                             {GyroODR::ODR_119HZ, "119 HZ"},
                                             {GyroODR::ODR_238HZ, "238 HZ"},
                                             {GyroODR::ODR_476HZ, "476 HZ"},
                                             {GyroODR::ODR_952HZ, "952 HZ"}});
  return map;
}

inline auto GyroScaleMap() {
  static std::map<GyroScale, cu::SpecInfo> map(
    {{GyroScale::SCALE_245DPS, {287.F, "245 DPS"}},  // DON"T CHANGE 0.000875
     {GyroScale::SCALE_500DPS, {574.F, "500 DPS"}},  // DON"T CHANGE 0.001750
     {GyroScale::SCALE_2000DPS, {2295.F, "2000 DPS"}}});  // DON"T CHANGE 0.0070
  return map;
}

inline auto MagODRMap() {
  static std::map<MagODR, std::string> map({{MagODR::ODR_0_625, "0.625 Hz"},
                                            {MagODR::ODR_1_25, "1.25 Hz"},
                                            {MagODR::ODR_2_50, "2.50 Hz"},
                                            {MagODR::ODR_5, "5 Hz"},
                                            {MagODR::ODR_10, "10 Hz"},
                                            {MagODR::ODR_20, "20 Hz"},
                                            {MagODR::ODR_40, "40 Hz"},
                                            {MagODR::ODR_80, "80 Hz"}});
  return map;
}

inline auto MagScaleMap() {
  static std::map<MagScale, cu::SpecInfo> map(
    {{MagScale::SCALE_4GS, {4.6F, "4 GS"}},      // DON"T CHANGE 0.00014
     {MagScale::SCALE_8GS, {9.5F, "8 GS"}},      // DON"T CHANGE 0.00029
     {MagScale::SCALE_12GS, {14.F, "12 GS"}},    // DON"T CHANGE 0.00043
     {MagScale::SCALE_16GS, {19.F, "16 GS"}}});  // DON"T CHANGE 0.00058
  return map;
}

inline auto MagXYModeMap() {
  static std::map<MagXYMode, std::string> map(
    {{MagXYMode::LOW, "LOW"},
     {MagXYMode::MEDIUM, "MEDIUM"},
     {MagXYMode::HIGH, "HIGH"},
     {MagXYMode::ULTRA_HIGH, "ULTRA HIGH"}});
  return map;
}

inline auto MagZModeMap() {
  static std::map<MagZMode, std::string> map(
    {{MagZMode::LOW, "LOW"},
     {MagZMode::MEDIUM, "MEDIUM"},
     {MagZMode::HIGH, "HIGH"},
     {MagZMode::ULTRA_HIGH, "ULTRA HIGH"}});
  return map;
}

inline auto MagOperatingModeMap() {
  static std::map<MagOperatingMode, std::string> map(
    {{MagOperatingMode::CONTINUOUS, "CONTINUOUS"},
     {MagOperatingMode::SINGLE, "SINGLE"},
     {MagOperatingMode::POWER_DOWN, "POWER DOWN"}});
  return map;
}

// Accelerometer and Gyroscope registers
namespace xg {
constexpr uint8_t WHO_AM_I_RESPONSE = 0x68;
constexpr uint8_t ACT_THS = 0x04;
constexpr uint8_t ACT_DUR = 0x05;
constexpr uint8_t INT_GEN_CFG_XL = 0x06;
constexpr uint8_t INT_GEN_THS_X_XL = 0x07;
constexpr uint8_t INT_GEN_THS_Y_XL = 0x08;
constexpr uint8_t INT_GEN_THS_Z_XL = 0x09;
constexpr uint8_t INT_GEN_DUR_XL = 0x0A;
constexpr uint8_t REFERENCE_G = 0x0B;
constexpr uint8_t INT1_CTRL = 0x0C;
constexpr uint8_t INT2_CTRL = 0x0D;
constexpr uint8_t WHO_AM_I = 0x0F;  // should return 0x68
constexpr uint8_t CTRL_REG1_G = 0x10;
constexpr uint8_t CTRL_REG2_G = 0x11;
constexpr uint8_t CTRL_REG3_G = 0x12;
constexpr uint8_t ORIENT_CFG_G = 0x13;
constexpr uint8_t INT_GEN_SRC_G = 0x14;
constexpr uint8_t OUT_TEMP_L = 0x15;
constexpr uint8_t OUT_TEMP_H = 0x16;
constexpr uint8_t STATUS_REG = 0x17;
constexpr uint8_t OUT_X_L_G = 0x18;
constexpr uint8_t OUT_X_H_G = 0x19;
constexpr uint8_t OUT_Y_L_G = 0x1A;
constexpr uint8_t OUT_Y_H_G = 0x1B;
constexpr uint8_t OUT_Z_L_G = 0x1C;
constexpr uint8_t OUT_Z_H_G = 0x1D;
constexpr uint8_t CTRL_REG4 = 0x1E;
constexpr uint8_t CTRL_REG5_XL = 0x1F;
constexpr uint8_t CTRL_REG6_XL = 0x20;
constexpr uint8_t CTRL_REG7_XL = 0x21;
constexpr uint8_t CTRL_REG8 = 0x22;
constexpr uint8_t CTRL_REG9 = 0x23;
constexpr uint8_t CTRL_REG10 = 0x24;
constexpr uint8_t INT_GEN_SRC_XL = 0x26;
constexpr uint8_t OUT_X_L_XL = 0x28;
constexpr uint8_t OUT_X_H_XL = 0x29;
constexpr uint8_t OUT_Y_L_XL = 0x2A;
constexpr uint8_t OUT_Y_H_XL = 0x2B;
constexpr uint8_t OUT_Z_L_XL = 0x2C;
constexpr uint8_t OUT_Z_H_XL = 0x2D;
constexpr uint8_t FIFO_CTRL = 0x2E;
constexpr uint8_t FIFO_SRC = 0x2F;
constexpr uint8_t INT_GEN_CFG_G = 0x30;
constexpr uint8_t INT_GEN_THS_XH_G = 0x31;
constexpr uint8_t INT_GEN_THS_XL_G = 0x32;
constexpr uint8_t INT_GEN_THS_YH_G = 0x33;
constexpr uint8_t INT_GEN_THS_YL_G = 0x34;
constexpr uint8_t INT_GEN_THS_ZH_G = 0x35;
constexpr uint8_t INT_GEN_THS_ZL_G = 0x36;
constexpr uint8_t INT_GEN_DUR_G = 0x37;

// Configuration bits Accelerometer and Gyroscope
constexpr uint8_t XEN_G = 0x08;
constexpr uint8_t YEN_G = 0x10;
constexpr uint8_t ZEN_G = 0x20;
constexpr uint8_t XEN_XL = 0x08;
constexpr uint8_t YEN_XL = 0x10;
constexpr uint8_t ZEN_XL = 0x20;
}  // namespace xg

namespace mag {
// Magnetometer registers
constexpr uint8_t WHO_AM_I_RESPONSE = 0x3D;
constexpr uint8_t OFFSET_X_REG_L_M = 0x05;
constexpr uint8_t OFFSET_X_REG_H_M = 0x06;
constexpr uint8_t OFFSET_Y_REG_L_M = 0x07;
constexpr uint8_t OFFSET_Y_REG_H_M = 0x08;
constexpr uint8_t OFFSET_Z_REG_L_M = 0x09;
constexpr uint8_t OFFSET_Z_REG_H_M = 0x0A;
constexpr uint8_t WHO_AM_I = 0x0F;  // should return 0x3D
constexpr uint8_t CTRL_REG1_M = 0x20;
constexpr uint8_t CTRL_REG2_M = 0x21;
constexpr uint8_t CTRL_REG3_M = 0x22;
constexpr uint8_t CTRL_REG4_M = 0x23;
constexpr uint8_t CTRL_REG5_M = 0x24;
constexpr uint8_t STATUS_REG_M = 0x27;
constexpr uint8_t OUT_X_L_M = 0x28;
constexpr uint8_t OUT_X_H_M = 0x29;
constexpr uint8_t OUT_Y_L_M = 0x2A;
constexpr uint8_t OUT_Y_H_M = 0x2B;
constexpr uint8_t OUT_Z_L_M = 0x2C;
constexpr uint8_t OUT_Z_H_M = 0x2D;
constexpr uint8_t INT_CFG_M = 0x30;
constexpr uint8_t INT_SRC_M = 0x31;
constexpr uint8_t INT_THS_L_M = 0x32;
constexpr uint8_t INT_THS_H_M = 0x33;
}  // namespace mag
}  // namespace sensors::lsm

// NOLINTBEGIN(misc-definitions-in-headers)
// NOLINTEND(misc-definitions-in-headers)
#endif  // SENSORS_LSM_LSM9DS1_REGISTER_MAP_HPP_
