/*
Written by Qiyong Mu (kylongmu@msn.com)
Adapted for Raspberry Pi by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
*/

#ifndef _MPU9250_H
#define _MPU9250_H

#include "spidev.h"
#include <core/sensors/module_sensor_imu.hpp>
#include <memory>
#include <vector>

class MPU9250 : public core::sensors::ImuSensorModule
{
 public:
  explicit MPU9250(const bool debug);

  void Initialize() override;
  bool Probe() override;
  void Update() override;

  void CalibAccelerometer();
  void CalibMagnetometer();

 protected:
  void RequestImu();
  static double ExtractTempreture(const std::vector<uint8_t>& response);

  core::utils::Vec3
  ExtractAccelerometer(const std::vector<uint8_t>& response) const;

  core::utils::Vec3
  ExtractGyroscope(const std::vector<uint8_t>& response) const;

  core::utils::Vec3
  ExtractMagnetometer(const std::vector<uint8_t>& response) const;

  core::utils::ImuData ExtractData(const std::vector<uint8_t>& response) const;

 private:
  uint8_t WriteReg(const uint8_t addr, const uint8_t data);
  uint8_t ReadReg(const uint8_t addr);
  std::vector<uint8_t> ReadRegs(const uint8_t addr, const uint32_t length);

  uint32_t SetGyroScale(const uint8_t scale);
  uint32_t SetAccelerometerScale(const uint8_t scale);

  float acc_divider_ {1.0f};
  float gyro_divider_{1.0f};

  int calib_data[3] = {1, 1, 1};
  float magnetometer_asa_[3] = {1.0, 1.0, 1.0};
  std::unique_ptr<SPIdev> spidev_{nullptr};
};

#endif  //_MPU9250_H

// Configuration bits MPU9250
constexpr uint8_t BIT_SLEEP = 0x40;
constexpr uint8_t BIT_H_RESET = 0x80;
constexpr uint8_t BITS_CLKSEL = 0x07;
constexpr uint8_t MPU_CLK_SEL_PLLGYROX = 0x01;
constexpr uint8_t MPU_CLK_SEL_PLLGYROZ = 0x03;
constexpr uint8_t MPU_EXT_SYNC_GYROX = 0x02;
constexpr uint8_t BITS_FS_250DPS = 0x00;
constexpr uint8_t BITS_FS_500DPS = 0x08;
constexpr uint8_t BITS_FS_1000DPS = 0x10;
constexpr uint8_t BITS_FS_2000DPS = 0x18;
constexpr uint8_t BITS_FS_2G = 0x00;
constexpr uint8_t BITS_FS_4G = 0x08;
constexpr uint8_t BITS_FS_8G = 0x10;
constexpr uint8_t BITS_FS_16G = 0x18;
constexpr uint8_t BITS_FS_MASK = 0x18;
constexpr uint8_t BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00;
constexpr uint8_t BITS_DLPF_CFG_188HZ = 0x01;
constexpr uint8_t BITS_DLPF_CFG_98HZ = 0x02;
constexpr uint8_t BITS_DLPF_CFG_42HZ = 0x03;
constexpr uint8_t BITS_DLPF_CFG_20HZ = 0x04;
constexpr uint8_t BITS_DLPF_CFG_10HZ = 0x05;
constexpr uint8_t BITS_DLPF_CFG_5HZ = 0x06;
constexpr uint8_t BITS_DLPF_CFG_2100HZ_NOLPF = 0x07;
constexpr uint8_t BITS_DLPF_CFG_MASK = 0x07;
constexpr uint8_t BIT_INT_ANYRD_2CLEAR = 0x10;
constexpr uint8_t BIT_RAW_RDY_EN = 0x01;
constexpr uint8_t BIT_I2C_IF_DIS = 0x10;

constexpr uint8_t READ_FLAG = 0x80;

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g 0.000061035156F   // 0.000061035156 g/LSB
#define MPU9250A_4g 0.000122070312F   // 0.000122070312 g/LSB
#define MPU9250A_8g 0.000244140625F  // 0.000244140625 g/LSB
#define MPU9250A_16g 0.000488281250F  // 0.000488281250 g/LSB

#define MPU9250G_250dps 0.007633587786F   // 0.007633587786 dps/LSB
#define MPU9250G_500dps 0.015267175572F   // 0.015267175572 dps/LSB
#define MPU9250G_1000dps 0.030487804878F  // 0.030487804878 dps/LSB
#define MPU9250G_2000dps 0.060975609756F  // 0.060975609756 dps/LSB

#define MPU9250M_4800uT 0.6F  // 0.6 uT/LSB

#define MPU9250T_85degC 0.002995177763F  // 0.002995177763 degC/LSB

#define Magnetometer_Sensitivity_Scale_Factor 0.15F
