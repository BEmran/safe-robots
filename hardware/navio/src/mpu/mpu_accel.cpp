#include <mpu/mpu_accel.hpp>
#include <mpu/spi.hpp>
#include <numeric>

namespace mpu
{
constexpr int AccelScales[4] = {2, 4, 8, 16};

MpuAccel::MpuAccel(const AccelConfig& config, const bool debug)
  : SensorModuleAccelerometer(SensorType, SensorName, debug), config_(config)
{
  AccelResolution();
  AccelSensitivity();
  printf("scale %u sensitivity_ %d resolution_ %f\n", Scale(), sensitivity_,
         resolution_);
}

void MpuAccel::AccelSensitivity()
{
  sensitivity_ = max_bit_val / AccelScales[Scale()];
}

void MpuAccel::AccelResolution()
{
  resolution_ = static_cast<float>(AccelScales[Scale()]) / max_bit_val;
}

void MpuAccel::Reset()
{
  mpu::ResetSensors();
}

void MpuAccel::Initialize()
{
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  GetSpi()->WriteRegister(SMPLRT_DIV, config_.sample_rate_divisor);

  // Set accelerometer full-scale range configuration
  uint8_t c1 = ReadRegister(ACCEL_CONFIG);
  // c = static_cast<uint8_t>(c & 0x1F_uc); // Clear self-test bits [7:5]
  c1 = static_cast<uint8_t>(c1 & 0xFC_uc);       // Clear AFS bits [4:3]
  c1 = static_cast<uint8_t>(c1 | Scale() << 3);  // Set full scale range for the
                                                 // gyro
  printf("Config %u\n", c1);
  // Write new ACCEL_CONFIG register value
  GetSpi()->WriteRegister(ACCEL_CONFIG, c1);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.13
  // kHz
  // get current ACCEL_CONFIG2 register value
  uint8_t c2 = ReadRegister(ACCEL_CONFIG2);
  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c2 = static_cast<uint8_t>(c2 & 0xF0_uc);
  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  c2 = static_cast<uint8_t>(c2 & 0x03_uc);
  // Write new ACCEL_CONFIG2 register value
  GetSpi()->WriteRegister(ACCEL_CONFIG2, c2);

  Delay(100);
}

uint8_t MpuAccel::Scale() const
{
  return static_cast<uint8_t>(config_.scale);
}

bool MpuAccel::Probe()
{
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 113)
  printf("Accel Whom %u\n", GetSpi()->ReadRegister(mpu::WHO_AM_I));
  return GetSpi()->ReadRegister(mpu::WHO_AM_I) == 0x71;
}

bool MpuAccel::Test()
{
  return false;
}

void MpuAccel::Update()
{
  const auto data = ReadAccelData();
  SetData(data);
}

uint8_t MpuAccel::ReadRegister(uint8_t reg)
{
  uint8_t buffer[1] = {0};
  GetSpi()->ReadRegisters(reg, 1, buffer);
  return buffer[0];
}

MpuAccel::SensorData MpuAccel::ReadAccelData() const
{
  MpuAccel::SensorData sensor_data;
  sensor_data.data = mpu::ReadData(mpu::ACCEL_XOUT_H, resolution_, GRAVITY);
  return sensor_data;
}

void MpuAccel::Calibrate()
{
}

}  // namespace mpu