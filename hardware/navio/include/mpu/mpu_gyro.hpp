#ifndef MPU_MPU_GYRO_HPP
#define MPU_MPU_GYRO_HPP

#include "mpu/my_utils.hpp"

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>

#include <stdint.h>
#include <unistd.h>
#include <array>

namespace mpu
{
using GyroData = core::utils::GyroData;
using SensorModuleGyroscope = core::sensors::SensorModuleAbs<GyroData>;

enum class GyroScale : uint8_t
{
  GFS_250DPS = 0x00,
  GFS_500DPS = 0x01,
  GFS_1000DPS = 0x02,
  GFS_2000DPS = 0x03
};

struct GyroConfig
{
  GyroScale scale;
  uint8_t sample_rate_divisor;
};

class MpuGyro : public SensorModuleGyroscope
{
  typedef GyroData SensorData;
  static constexpr auto SensorType = core::sensors::SensorModuleType::GYROSCOPE;
  static constexpr const char* SensorName = "Gyro";

 public:
  MpuGyro(const GyroConfig& config, const bool debug);

  void Initialize() override;

  bool Probe() override;

  bool Test() override;

  void Update() override;

  void Calibrate() override;

 protected:
  void Reset();

  SensorData ReadGyroscope() const;
  static std::array<int16_t, 3> ReadGyroFullBits();
  void GyroSensitivity();
  void GyroResolution();

 private:
  uint8_t Scale() const;

  GyroConfig config_;
  int sensitivity_{1};     // sensitivity
  float resolution_{1.0};  // bit resolution
  // std::array<float, 3> sensitivity_calibration_ = {
  //     1.0F, 1.0F, 1.0F};  // factory calibration
  // std::array<float, 3> bias_correction_ = {0.0F, 0.0F,
  //                                          0.0F};  // hard iron correction
  // std::array<float, 3> scale_correction_ = {1.0F, 1.0F,
  //                                           1.0F};  // soft iron correction
};
}  // namespace mpu
#endif  // MPU_MPU_GYRO_HPP