// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef SENSORS_MPU_MPU9250_REGISTER_MAP_HPP_
#define SENSORS_MPU_MPU9250_REGISTER_MAP_HPP_

#include <cstdint>
#include <map>
#include <string>

#include "sensors/common/utils.hpp"

namespace sensors::mpu {
namespace cu = common::utils;

constexpr int kMaxBitVal = 32760;  // Measurement range is from -32760 ~ +32760
                                   // decimal in 16-bit output.
constexpr int kMaxUTesla = 4912;   // Magnetic flux density in micro Tesla
constexpr auto TempSensitivity = 333.87F;
const auto kTempBias = 21.F;
const auto kTempOffset = 21.F;

using namespace common::utils::literals;  // NOLINT [build/namespaces_literals]
                                          // TODO(Bara)

enum class GyroScale : uint8_t {
  FS_250DPS = 0x00_uc,
  FS_500DPS = 0x08_uc,
  FS_1000DPS = 0x10_uc,
  FS_2000DPS = 0x18_uc
};

enum class GyroBandWidthHz : uint8_t {
  BW_250HZ = 0x00_uc,
  BW_184HZ = 0x01_uc,
  BW_92HZ = 0x02_uc,
  BW_41HZ = 0x03_uc,
  BW_20HZ = 0x04_uc,
  BW_10HZ = 0x05_uc,
  BW_5HZ = 0x06_uc,
  BW_3600HZ = 0x07_uc
};

enum class AccelScale : uint8_t {
  FS_2G = 0x00_uc,
  FS_4G = 0x08_uc,
  FS_8G = 0x10_uc,
  FS_16G = 0x18_uc
};

enum class AccelBandWidthHz : uint8_t {
  BW_218HZ = 0x01_uc,
  BW_99HZ = 0x02_uc,
  BW_44HZ = 0x03_uc,
  BW_21HZ = 0x04_uc,
  BW_10HZ = 0x05_uc,
  BW_5HZ = 0x06_uc
};

enum class MagScale : uint8_t { FS_14BITS = 0x00_uc, FS_16BITS = 0x10_uc };

enum class MagMode : uint8_t {
  POWER_DOWN = 0x00_uc,
  SINGLE_MEASURE = 0x01_uc,
  CONTINUES_8HZ = 0x02_uc,
  EXTERNAL_TRIGGER = 0x04_uc,
  CONTINUES_100HZ = 0x06_uc,
  SELF_TEST = 0x08_uc,
  FUSE_ROM_ACCESS = 0x0F_uc
};

inline auto AccelBWMap() {
  static std::map<AccelBandWidthHz, std::string> map(
    {{AccelBandWidthHz::BW_218HZ, "218 HZ"},
     {AccelBandWidthHz::BW_99HZ, "99 HZ"},
     {AccelBandWidthHz::BW_44HZ, "44 HZ"},
     {AccelBandWidthHz::BW_21HZ, "21 HZ"},
     {AccelBandWidthHz::BW_10HZ, "10 HZ"},
     {AccelBandWidthHz::BW_5HZ, "5 HZ"}});
  return map;
}

inline auto AccelScaleMap() {
  static std::map<AccelScale, cu::SpecInfo> map(
    {{AccelScale::FS_2G, {2.F, "2G"}},
     {AccelScale::FS_4G, {4.F, "4G"}},
     {AccelScale::FS_8G, {8.F, "8G"}},
     {AccelScale::FS_16G, {16.F, "16G"}}});
  return map;
}

inline auto GyroBWMap() {
  static std::map<GyroBandWidthHz, std::string> map(
    {{GyroBandWidthHz::BW_250HZ, "250 HZ"},
     {GyroBandWidthHz::BW_184HZ, "184 HZ"},
     {GyroBandWidthHz::BW_92HZ, "92 HZ"},
     {GyroBandWidthHz::BW_41HZ, "41 HZ"},
     {GyroBandWidthHz::BW_20HZ, "20 HZ"},
     {GyroBandWidthHz::BW_10HZ, "10 HZ"},
     {GyroBandWidthHz::BW_5HZ, "5 HZ"},
     {GyroBandWidthHz::BW_3600HZ, "3600 HZ"}});
  return map;
}

inline auto GyroScaleMap() {
  std::map<GyroScale, cu::SpecInfo> map(
    {{GyroScale::FS_250DPS, {250.F, "250 DPS"}},
     {GyroScale::FS_500DPS, {500.F, "500 DPS"}},
     {GyroScale::FS_1000DPS, {1000.F, "1000 DPS"}},
     {GyroScale::FS_2000DPS, {2000.F, "2000 DPS"}}});
  return map;
}

inline auto MagScaleMap() {
  std::map<MagScale, cu::SpecInfo> map(
    {{MagScale::FS_14BITS, {0.25F * kMaxUTesla, "14 BITS"}},
     {MagScale::FS_16BITS, {1.00F * kMaxUTesla, "16 BITS"}}});
  return map;
}

inline auto MagModeMap() {
  static std::map<MagMode, std::string> map(
    {{MagMode::POWER_DOWN, "POWER DOWN"},
     {MagMode::SINGLE_MEASURE, "SINGLE MEASUREMENT"},
     {MagMode::CONTINUES_8HZ, "CONTINUES 8HZ"},
     {MagMode::EXTERNAL_TRIGGER, "EXTERNAL TRIGGER"},
     {MagMode::CONTINUES_100HZ, "CONTINUES 100HZ"},
     {MagMode::SELF_TEST, "SELF TEST"},
     {MagMode::FUSE_ROM_ACCESS, "FUSE ROM ACCESS"}});
  return map;
}

// NOLINTBEGIN(misc-definitions-in-headers)

// NOLINTEND(misc-definitions-in-headers)

// mpu9250 register map
namespace mpu9250 {
constexpr uint8_t WHO_AM_I_RESPONSE = 0x71;
constexpr uint8_t ADDRESS = 0x68;
constexpr uint8_t SELF_TEST_X_ACCEL = 0x0D;
constexpr uint8_t SELF_TEST_Y_ACCEL = 0x0E;
constexpr uint8_t SELF_TEST_Z_ACCEL = 0x0F;
constexpr uint8_t SELF_TEST_A = 0x10;
constexpr uint8_t XG_OFFSET_H = 0x13;
constexpr uint8_t XG_OFFSET_L = 0x14;
constexpr uint8_t YG_OFFSET_H = 0x15;
constexpr uint8_t YG_OFFSET_L = 0x16;
constexpr uint8_t ZG_OFFSET_H = 0x17;
constexpr uint8_t ZG_OFFSET_L = 0x18;
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t I2C_MST_EN = 0x20;
constexpr uint8_t ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t LP_ACCEL_ODR = 0x1E;
constexpr uint8_t MOT_THR = 0x1F;
constexpr uint8_t MOT_DUR = 0x20;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t ZMOT_THR = 0x21;
constexpr uint8_t ZRMOT_DUR = 0x22;
constexpr uint8_t FIFO_EN = 0x23;
constexpr uint8_t I2C_MST_CTRL = 0x24;
constexpr uint8_t I2C_SLV0_ADDR = 0x25;
constexpr uint8_t I2C_SLV0_REG = 0x26;
constexpr uint8_t I2C_SLV0_CTRL = 0x27;
constexpr uint8_t I2C_SLV1_ADDR = 0x28;
constexpr uint8_t I2C_SLV1_REG = 0x29;
constexpr uint8_t I2C_SLV1_CTRL = 0x2A;
constexpr uint8_t I2C_SLV2_ADDR = 0x2B;
constexpr uint8_t I2C_SLV2_REG = 0x2C;
constexpr uint8_t I2C_SLV2_CTRL = 0x2D;
constexpr uint8_t I2C_SLV3_ADDR = 0x2E;
constexpr uint8_t I2C_SLV3_REG = 0x2F;
constexpr uint8_t I2C_SLV3_CTRL = 0x30;
constexpr uint8_t I2C_SLV4_ADDR = 0x31;
constexpr uint8_t I2C_SLV4_REG = 0x32;
constexpr uint8_t I2C_SLV4_DO = 0x33;
constexpr uint8_t I2C_SLV4_CTRL = 0x34;
constexpr uint8_t I2C_SLV4_DI = 0x35;
constexpr uint8_t I2C_MST_STATUS = 0x36;
constexpr uint8_t INT_PIN_CFG = 0x37;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t DMP_INT_STATUS = 0x39;  // Check DMP interrupt
constexpr uint8_t INT_STATUS = 0x3A;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t ACCEL_XOUT_L = 0x3C;
constexpr uint8_t ACCEL_YOUT_H = 0x3D;
constexpr uint8_t ACCEL_YOUT_L = 0x3E;
constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t ACCEL_ZOUT_L = 0x40;
constexpr uint8_t TEMP_OUT_H = 0x41;
constexpr uint8_t TEMP_OUT_L = 0x42;
constexpr uint8_t GYRO_XOUT_H = 0x43;
constexpr uint8_t GYRO_XOUT_L = 0x44;
constexpr uint8_t GYRO_YOUT_H = 0x45;
constexpr uint8_t GYRO_YOUT_L = 0x46;
constexpr uint8_t GYRO_ZOUT_H = 0x47;
constexpr uint8_t GYRO_ZOUT_L = 0x48;
constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
constexpr uint8_t EXT_SENS_DATA_01 = 0x4A;
constexpr uint8_t EXT_SENS_DATA_02 = 0x4B;
constexpr uint8_t EXT_SENS_DATA_03 = 0x4C;
constexpr uint8_t EXT_SENS_DATA_04 = 0x4D;
constexpr uint8_t EXT_SENS_DATA_05 = 0x4E;
constexpr uint8_t EXT_SENS_DATA_06 = 0x4F;
constexpr uint8_t EXT_SENS_DATA_07 = 0x50;
constexpr uint8_t EXT_SENS_DATA_08 = 0x51;
constexpr uint8_t EXT_SENS_DATA_09 = 0x52;
constexpr uint8_t EXT_SENS_DATA_10 = 0x53;
constexpr uint8_t EXT_SENS_DATA_11 = 0x54;
constexpr uint8_t EXT_SENS_DATA_12 = 0x55;
constexpr uint8_t EXT_SENS_DATA_13 = 0x56;
constexpr uint8_t EXT_SENS_DATA_14 = 0x57;
constexpr uint8_t EXT_SENS_DATA_15 = 0x58;
constexpr uint8_t EXT_SENS_DATA_16 = 0x59;
constexpr uint8_t EXT_SENS_DATA_17 = 0x5A;
constexpr uint8_t EXT_SENS_DATA_18 = 0x5B;
constexpr uint8_t EXT_SENS_DATA_19 = 0x5C;
constexpr uint8_t EXT_SENS_DATA_20 = 0x5D;
constexpr uint8_t EXT_SENS_DATA_21 = 0x5E;
constexpr uint8_t EXT_SENS_DATA_22 = 0x5F;
constexpr uint8_t EXT_SENS_DATA_23 = 0x60;
constexpr uint8_t MOT_DETECT_STATUS = 0x61;
constexpr uint8_t I2C_SLV0_DO = 0x63;
constexpr uint8_t I2C_SLV1_DO = 0x64;
constexpr uint8_t I2C_SLV2_DO = 0x65;
constexpr uint8_t I2C_SLV3_DO = 0x66;
constexpr uint8_t I2C_MST_DELAY_CTRL = 0x67;
constexpr uint8_t SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t MOT_DETECT_CTRL = 0x69;
constexpr uint8_t USER_CTRL = 0x6A;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t DMP_BANK = 0x6D;
constexpr uint8_t DMP_RW_PNT = 0x6E;
constexpr uint8_t DMP_REG = 0x6F;
constexpr uint8_t DMP_REG_1 = 0x70;
constexpr uint8_t DMP_REG_2 = 0x71;
constexpr uint8_t FIFO_COUNTH = 0x72;
constexpr uint8_t FIFO_COUNTL = 0x73;
constexpr uint8_t FIFO_R_W = 0x74;
constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t XA_OFFSET_H = 0x77;
constexpr uint8_t XA_OFFSET_L = 0x78;
constexpr uint8_t YA_OFFSET_H = 0x7A;
constexpr uint8_t YA_OFFSET_L = 0x7B;
constexpr uint8_t ZA_OFFSET_H = 0x7D;
constexpr uint8_t ZA_OFFSET_L = 0x7E;
constexpr uint8_t I2C_SLV0_EN = 0x80;
constexpr uint8_t I2C_READ_FLAG = 0x80;
}  // namespace mpu9250

// Magnetometer register map
namespace ak8963 {
constexpr uint8_t WHO_AM_I_RESPONSE = 0x48;
constexpr uint8_t I2C_ADDR = 0x0C;
constexpr uint8_t WHO_AM_I = 0x00;
constexpr uint8_t INFO = 0x01;
constexpr uint8_t ST1 = 0x02;
constexpr uint8_t XOUT_L = 0x03;
constexpr uint8_t XOUT_H = 0x04;
constexpr uint8_t YOUT_L = 0x05;
constexpr uint8_t YOUT_H = 0x06;
constexpr uint8_t ZOUT_L = 0x07;
constexpr uint8_t ZOUT_H = 0x08;
constexpr uint8_t ST2 = 0x09;
constexpr uint8_t CNTL = 0x0A;
constexpr uint8_t ASTC = 0x0C;
constexpr uint8_t I2CDIS = 0x0F;
constexpr uint8_t ASAX = 0x10;
constexpr uint8_t ASAY = 0x11;
constexpr uint8_t ASAZ = 0x12;
}  // namespace ak8963
}  // namespace sensors::mpu
#endif  // SENSORS_MPU_MPU9250_REGISTER_MAP_HPP_
