#ifndef MPU_MPU9250_HPP
#define MPU_MPU9250_HPP

#include "mpu/my_utils.hpp"

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>

#include <stdint.h>
#include <unistd.h>
#include <array>

namespace mpu
{

using ImuData = core::utils::ImuData;
using ImuSensorModule = core::sensors::SensorModuleAbs<ImuData>;

enum class GyroScale : uint8_t
{
  GFS_250DPS = 0x00,
  GFS_500DPS = 0x08,
  GFS_1000DPS = 0x10,
  GFS_2000DPS = 0x18
};

enum class GyroBandWidthHz : uint8_t
{
  GBW_250HZ = 0x00,  // Gyro sf: 8 kHz delay: 0.97 ms, Temperature BW: 4000 Hz
  GBW_184HZ = 0x01,  // Gyro sf: 1 kHz delay: 2.9 ms, Temperature BW: 188 Hz
  GBW_92HZ = 0x02,   // Gyro sf: 1 kHz delay: 3.9 ms, Temperature BW: 98 Hz
  GBW_41HZ = 0x03,   // Gyro sf: 1 kHz delay: 5.9 ms, Temperature BW: 42 Hz
  GBW_20HZ = 0x04,   // Gyro sf: 1 kHz delay: 9.9 ms, Temperature BW: 20 Hz
  GBW_10HZ = 0x05,   // Gyro sf: 1 kHz delay: 17.85 ms, Temperature BW: 10 Hz
  GBW_5HZ = 0x06,    // Gyro sf: 1 kHz delay: 33.48 ms, Temperature BW: 5 Hz
  GBW_3600HZ = 0x07  // Gyro sf: 8 kHz delay: 0.17 ms, Temperature BW: 4000 Hz
};

enum class AccelBandWidthHz : uint8_t
{
  GBW_218HZ = 0x01,  // sf: 1 kHz delay: 1.88 ms
  GBW_99HZ = 0x02,   // sf: 1 kHz delay: 2.88 ms
  GBW_44HZ = 0x03,   // sf: 1 kHz delay: 4.88 ms
  GBW_21HZ = 0x04,   // sf: 1 kHz delay: 8.78 ms
  GBW_10HZ = 0x05,   // sf: 1 kHz delay: 16.83 ms
  GBW_5HZ = 0x06,    // sf: 1 kHz delay: 32.48 ms
};

enum class AccelScale : uint8_t
{
  AFS_2G = 0x00,
  AFS_4G = 0x08,
  AFS_8G = 0x10,
  AFS_16G = 0x18
};

enum class MagScale : uint8_t
{
  MFS_14BITS = 0x00,  // 0.6 mG per LSB
  MFS_16BITS = 0x10   // 0.15 mG per LSB
};

enum class MagMode : uint8_t
{
  CONTINUES_8HZ_MODE = 0x02,
  CONTINUES_100HZ_MODE = 0x06
};

struct Config
{
  AccelScale accel_scale = AccelScale::AFS_16G;
  AccelBandWidthHz accel_bw = AccelBandWidthHz::GBW_44HZ;
  GyroScale gyro_scale = GyroScale::GFS_2000DPS;
  GyroBandWidthHz gyro_bw = GyroBandWidthHz::GBW_184HZ;
  MagScale mag_scale = MagScale::MFS_16BITS;
  MagMode mag_mode = MagMode::CONTINUES_100HZ_MODE;
  uint8_t sample_rate_divisor = 4;
};

class Mpu9250 : public ImuSensorModule
{
  using AccelData = core::utils::AccelData;
  using GyroData = core::utils::GyroData;
  using MagData = core::utils::MagData;
  using TemperatureData = core::utils::TemperatureData;
  static constexpr auto SensorType = core::sensors::SensorModuleType::IMU;
  static constexpr const char* SensorName = "mpu9250";

 public:
  Mpu9250(const Config& config, const bool debug);

  void Initialize() override;

  bool Probe() override;

  bool Test() override;

  void Update() override;

  void Calibrate() override;

 protected:
  static void Reset();
  
  static ImuData ReadAll();
  static std::array<int16_t, 7> ReadAccelGyroTemp();
  static std::array<int16_t, 3> ReadMagnetometer();
  static AccelData ExtractAccelerometer(const std::array<int16_t, 7>& full_bits);
  static GyroData ExtractGyroscope(const std::array<int16_t, 7>& full_bits);
  static TemperatureData ExtractTemperature(const std::array<int16_t, 7>& full_bits);
  static MagData ExtractMagnetometer(const std::array<int16_t, 3>& full_bits);

  void InitializeAccel() const;
  void InitializeGyro() const;
  void InitializeMag() const;

  void ExtractSensitivityAdjustmentValues();

  static uint8_t ReadRegister(const uint8_t reg);
  static void ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t* dest);
  static uint8_t ReadAK8963Register(const uint8_t reg);
  static void RequestReadAK8963Registers(const uint8_t reg, const uint8_t count);
  static void ReadAK8963Registers(const uint8_t reg, const uint8_t count, uint8_t* dest);
  static void WriteRegister(const uint8_t reg, const uint8_t data);
  static void WriteAK8963Register(const uint8_t reg, const uint8_t data);

 private:
  typedef enum
  {
    POWER_DOWN_MODE = 0x00,
    SINGLE_MEASUREMENT_MODE = 0x01,
    EXTERNAL_TRIGGER_MODE = 0x04,
    SELF_TEST_MODE = 0x08,
    FUSE_ROM_ACCESS_MODE = 0x0F
  } mag_mode_t;

  Config config_;

  std::array<float, 3> sensitivity_calibration_ = {
      1.0F, 1.0F, 1.0F};  // factory calibration
  // std::array<float, 3> bias_correction_ = {0.0F, 0.0F,
  //                                          0.0F};  // hard iron correction
  // std::array<float, 3> scale_correction_ = {1.0F, 1.0F,
  //                                           1.0F};  // soft iron correction
};
}  // namespace mpu
#endif  // MPU_MPU9250_HPP