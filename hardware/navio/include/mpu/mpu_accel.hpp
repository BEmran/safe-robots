#ifndef MPU_MPU_ACCEL_HPP
#define MPU_MPU_ACCEL_HPP

#include "mpu/my_utils.hpp"

#include <core/sensors/module_sensor.hpp>
#include <core/utils/data.hpp>

#include <stdint.h>
#include <unistd.h>
#include <array>

namespace mpu
{

using AccelData = core::utils::AccelData;
using SensorModuleAccelerometer = core::sensors::SensorModuleAbs<AccelData>;

enum class AccelScale : uint8_t
{
  AFS_2G = 0x00,
  AFS_4G = 0x01,
  AFS_8G = 0x02,
  AFS_16G = 0x03
};

struct AccelConfig
{
  AccelScale scale;
  uint8_t sample_rate_divisor;
};

class MpuAccel : public SensorModuleAccelerometer
{
  typedef AccelData SensorData;
  static constexpr auto SensorType =
      core::sensors::SensorModuleType::ACCELEROMETER;
  static constexpr const char* SensorName = "Accel";

 public:
  MpuAccel(const AccelConfig& config, const bool debug);

  void Initialize() override;

  bool Probe() override;

  bool Test() override;

  void Update() override;

  void Calibrate() override;

 protected:
  void Reset();

  SensorData ReadAccelData() const;
  void AccelSensitivity();
  void AccelResolution();

 private:
  uint8_t Scale() const;

  static uint8_t ReadRegister(uint8_t reg);

  AccelConfig config_;
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
#endif  // MPU_MPU_ACCEL_HPP