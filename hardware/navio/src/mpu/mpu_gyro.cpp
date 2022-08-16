#include <mpu/mpu_gyro.hpp>
#include <mpu/spi.hpp>
#include <numeric>

namespace mpu
{
constexpr int GyroScales[4] = {250, 500, 1000, 2000};

MpuGyro::MpuGyro(const GyroConfig& config, const bool debug)
  : SensorModuleGyroscope(SensorType, SensorName, debug)
  , config_(config)
{
  GyroResolution();
  GyroSensitivity();
  printf("scale %u sensitivity_ %d resolution_ %f\n", Scale(), sensitivity_, resolution_);
}

void MpuGyro::GyroSensitivity()
{
  sensitivity_ = max_bit_val / GyroScales[Scale()];
}

void MpuGyro::GyroResolution()
{
  resolution_ = static_cast<float>(GyroScales[Scale()]) / max_bit_val;
}

void MpuGyro::Reset()
{
  mpu::ResetSensors();
}

void MpuGyro::Initialize()
{
  Reset();
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively; minimum delay time for this setting is 5.9 ms, which means
  // sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
  // kHz, or 1 kHz

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  GetSpi()->WriteRegister(mpu::SMPLRT_DIV, config_.sample_rate_divisor);

  // determined inset in CONFIG above
  // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
  GetSpi()->WriteRegister(mpu::CONFIG, 0x01);  

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  // get current GYRO_CONFIG register value
  uint8_t c = GetSpi()->ReadRegister(mpu::GYRO_CONFIG);
  // c = static_cast<uint8_t>(c & 0x1F_uc); // Clear self-test bits [7:5
  c = static_cast<uint8_t>(c & 0xFC_uc);    // Clear Fchoice bits [1:0]
  c = static_cast<uint8_t>(c & 0xE7_uc);    // Clear AFS bits [4:3]
  c = static_cast<uint8_t>(c | Scale() << 3);  // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
  // 1:0 of GYRO_CONFIG
  // Write new GYRO_CONFIG value to register
  printf("Config %u\n", c);
  GetSpi()->WriteRegister(mpu::GYRO_CONFIG, c);

  Delay(100);
}

uint8_t MpuGyro::Scale() const
{
  return static_cast<uint8_t>(config_.scale);
}

bool MpuGyro::Probe()
{
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 113)
  printf("Gyro Whom %u\n",  GetSpi()->ReadRegister(mpu::WHO_AM_I));
  return GetSpi()->ReadRegister(mpu::WHO_AM_I) == 0x71;
}

bool MpuGyro::Test()
{
  return false;
}

void MpuGyro::Update()
{
  const auto data = ReadGyroscope();
  SetData(data);
}

MpuGyro::SensorData MpuGyro::ReadGyroscope() const
{
  MpuGyro::SensorData sensor_data;
  sensor_data.data = mpu::ReadData(mpu::GYRO_XOUT_H, resolution_, PI / 180.0F);
  return sensor_data;
}

void MpuGyro::Calibrate()
{
}

}  // namespace mpu