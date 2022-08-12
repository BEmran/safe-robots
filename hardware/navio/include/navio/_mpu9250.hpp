#ifndef HARDWARE_NAVIO2_MPU9250_HPP
#define HARDWARE_NAVIO2_MPU9250_HPP

#include <core/sensors/module_sensor_imu.hpp>
using core::sensors::ImuSensorModule;

// Configuration bits MPU9250
constexpr uint8_t BIT_SLEEP = 0x40;
constexpr uint8_t BIT_H_RESET = 0x80;
constexpr uint8_t BITS_CLKSEL = 0x07;
constexpr uint8_t MPU_CLK_SEL_PLLGYROX = 0x01;
constexpr uint8_t MPU_CLK_SEL_PLLGYROZ = 0x03;
constexpr uint8_t MPU_EXT_SYNC_GYROX = 0x02;
// namespace Gyroscope
// {
//     enum class FullScaleRange : uint8_t
//     {
//         FSR_250DPS = 0x00,
//         FSR_500DPS = 0x08,
//         FSR_1000DPS = 0x10,
//         FSR_2000DPS = 0x18
//     };
// }

// namespace Accelerometer
// {
//     enum class FullScaleRange : uint8_t
//     {
//         FSR_2G = 0x00,
//         FSR_4G = 0x08,
//         FSR_8G = 0x10,
//         FSR_16G = 0x18
//     };
// }

class MPU9250 : public ImuSensorModule
{
 public:
  MPU9250() = default;

  bool initialize();
  bool probe();
  void update();

  void CalibrateAcceleometer();
  void CalibrateMagnetometer();

 private:
  uint8_t WriteReg(uint8_t WriteAddr, uint8_t WriteData);
  uint8_t ReadReg(uint8_t ReadAddr);
  void ReadRegs(uint8_t ReadAddr, uint8_t* ReadBuf, uint8_t Bytes);

  unsigned int SetGyroCalibrationScale(uint8_t scale);
  unsigned int SetAcceleometerScale(uint8_t scale);

  float acc_divider = 1.0;
  float gyro_divider = 1.0;
  int calib_data[3] = {0, 0, 0};
  float magnetometer_asa[3] = {1.0, 1.0, 1.0};
};

#endif  // HARDWARE_NAVIO2_MPU9250_HPP