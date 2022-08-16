#include "mpu/mpu9250.hpp"
#include "mpu/spi.hpp"

#include <numeric>
#include <vector>
#include <map>

namespace mpu
{
namespace
{
constexpr int GyroScales[4] = {250, 500, 1000, 2000};
constexpr int AccelScales[4] = {2, 4, 8, 16};

constexpr auto Accel = core::sensors::SensorModuleType::ACCELEROMETER;
constexpr auto Gyro = core::sensors::SensorModuleType::GYROSCOPE;
constexpr auto Mag = core::sensors::SensorModuleType::MAGNETOMETER;

std::map<core::sensors::SensorModuleType, Post> post_map = {{Accel, Post()},
                                                            {Gyro, Post()},
                                                            {Mag, Post()}};

std::map<AccelScale, float> accel_scale_map = {{AccelScale::AFS_2G, 2.F},
                                               {AccelScale::AFS_4G, 4.F},
                                               {AccelScale::AFS_8G, 8.F},
                                               {AccelScale::AFS_16G, 16.F}};

std::map<GyroScale, float> gyro_scale_map = {{GyroScale::GFS_250DPS, 250.F},
                                             {GyroScale::GFS_500DPS, 500.F},
                                             {GyroScale::GFS_1000DPS, 1000.F},
                                             {GyroScale::GFS_2000DPS, 2000.F}};

std::map<MagScale, float> mag_scale_map = {{MagScale::MFS_14BITS, 0.25F},
                                           {MagScale::MFS_16BITS, 1.00F}};

float Sensitivity(const float scale)
{
  return max_bit_val / scale;
}

float Resolution(const float scale)
{
  return scale / max_bit_val;
}

Post CratePost(const float scale, const float unit)
{
  const auto sen = Sensitivity(scale);
  const auto res = Resolution(scale);

  std::cout << "scale: " << scale << ", sensitivity: " << sen
            << ", resolution: " << res << " unit:" << unit << std::endl;

  return Post(sen, res, unit);
}

}  // namespace

Mpu9250::Mpu9250(const Config& config, const bool debug)
  : ImuSensorModule(SensorType, SensorName, debug), config_(config)
{
  std::cout << "acc scale:" << accel_scale_map[config.accel_scale] << std::endl;
  std::cout << "gro scale:" << gyro_scale_map[config.gyro_scale] << std::endl;
  std::cout << "mag scale:" << mag_scale_map[config.mag_scale] << std::endl;

  post_map[Accel] = CratePost(accel_scale_map[config.accel_scale], GRAVITY);
  post_map[Gyro] = CratePost(gyro_scale_map[config.gyro_scale], PI / 180.0F);
  post_map[Mag] = CratePost(mag_scale_map[config.mag_scale], max_micro_tesla);
}

void Mpu9250::Reset()
{
  // wake up device, clear sleep mode bit (6)
  // mpu::WriteMPURegister(mpu9250::PWR_MGMT_1, 0x00);

  // reset all registers to reset bit (7)
  mpu::WriteMPURegister(mpu9250::PWR_MGMT_1, 0x80);

  Delay(100);  // Wait for all registers to reset

  // Auto select clock source to be PLL gyroscope reference, else internal 20MHz
  WriteMPURegister(mpu9250::PWR_MGMT_1, 0x01);
  Delay(200);
}

bool Mpu9250::Probe()
{
  // check MPU WHO AM I register, expected value is 0x48 (decimal 113)
  if (mpu::ReadMPURegister(mpu9250::WHO_AM_I) != 0x71)
  {
    std::cout << "Bad IMU device ID" << std::endl;
    return false;
  }

  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  if (mpu::ReadAK8963Register(ak8963::WHO_AM_I) != 0x48)
  {
    std::cout << "Bad Magnetometer device ID" << std::endl;
    return false;
  }

  std::cout << "MPU9250 online!" << std::endl;
  return true;
}

bool Mpu9250::Test()
{
  return false;
}

void Mpu9250::Initialize()
{
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  mpu::WriteMPURegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  InitializeAccel();
  InitializeGyro();
  InitializeMag();
  ExtractSensitivityAdjustmentValues();
}

void Mpu9250::InitializeAccel() const
{
  // Set accelerometer full-scale range configuration
  uint8_t c1 = mpu::ReadMPURegister(mpu9250::ACCEL_CONFIG);
  c1 = static_cast<uint8_t>(c1 & 0xF8_uc);  // Clear AFS bits [4:3]
  const uint8_t configured_scale = static_cast<uint8_t>(config_.accel_scale);
  c1 = static_cast<uint8_t>(c1 | configured_scale);
  // Write new ACCEL_CONFIG register value
  mpu::WriteMPURegister(mpu9250::ACCEL_CONFIG, c1);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.03
  // kHz
  // get current ACCEL_CONFIG2 register value
  uint8_t c2 = mpu::ReadMPURegister(mpu9250::ACCEL_CONFIG2);
  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c2 = static_cast<uint8_t>(c2 & 0xF0_uc);
  // Set accelerometer rate and bandwidth
  const uint8_t configured_bw = static_cast<uint8_t>(config_.accel_bw);
  c2 = static_cast<uint8_t>(c2 & configured_bw);
  // Write new ACCEL_CONFIG2 register value
  mpu::WriteMPURegister(mpu9250::ACCEL_CONFIG2, c2);
  printf("acc Config1 %u\n", c1);
  printf("acc Config2 %u\n", c2);

  Delay(100);
}

void Mpu9250::InitializeGyro() const
{
  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively; minimum delay time for this setting is 5.9 ms, which means
  // sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
  // kHz, or 1 kHz

  // determined inset in CONFIG above to use DLPF
  mpu::WriteMPURegister(mpu9250::CONFIG, 0x01);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  // get current GYRO_CONFIG register value
  uint8_t c = mpu::ReadMPURegister(mpu9250::GYRO_CONFIG);
  // Clear Fchoice bits [1:0] and AFS bits [4:3]
  c = static_cast<uint8_t>(c & 0xE4_uc);
  const uint8_t configured_scale = static_cast<uint8_t>(config_.gyro_scale);
  c = static_cast<uint8_t>(c | configured_scale);
  // Write new GYRO_CONFIG value to register
  mpu::WriteMPURegister(mpu9250::GYRO_CONFIG, c);
  printf("gyro Config %u\n", c);
  Delay(100);
}

void Mpu9250::InitializeMag() const
{
  // enable master mode
  WriteMPURegister(mpu9250::USER_CTRL, mpu9250::I2C_MST_EN);
  // I2C configuration multi-master IIC 400KHz
  WriteMPURegister(mpu9250::I2C_MST_CTRL, 0x0D);

  mpu::WriteAK8963Register(ak8963::CNTL, mag_mode_t::POWER_DOWN_MODE);
  Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Scale bit 4 to 1 or (0) to enable 16 or (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // Set magnetometer data resolution and sample ODR+
  const auto configured_scale = static_cast<uint8_t>(config_.mag_scale);
  const auto configured_mode = static_cast<uint8_t>(config_.mag_mode);
  mpu::WriteAK8963Register(ak8963::CNTL, configured_mode | configured_scale);
  Delay(10);
  printf("mag Config %d\n", configured_mode | configured_scale);
}

void Mpu9250::ExtractSensitivityAdjustmentValues()
{
  mpu::WriteAK8963Register(ak8963::CNTL, mag_mode_t::POWER_DOWN_MODE);
  Delay(10);
  mpu::WriteAK8963Register(ak8963::CNTL, mag_mode_t::FUSE_ROM_ACCESS_MODE);
  Delay(10);
  // Read the x-, y-, and z-axis calibration values
  uint8_t asa_values[3];
  mpu::ReadAK8963Registers(ak8963::ASAX, 3, asa_values);
  // calculate xyz-axis manufacture sensitivity adjustment values
  for (size_t i = 0; i < 3; i++)
  {
    sensitivity_calibration_[i] =
        static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;
  }
  // debug
  std::cout << "sensitivity [" << sensitivity_calibration_[0] << ", "
            << sensitivity_calibration_[1] << ", "
            << sensitivity_calibration_[2] << "]" << std::endl;
}

void Mpu9250::Update()
{
  ImuData data = ReadAll();
  SetData(data);
}

ImuData Mpu9250::ReadAll()
{
  auto agt_full_bits = ReadAccelGyroTemp();

  ImuData imu;
  imu.accel = ExtractAccelerometer(agt_full_bits);
  imu.gyro = ExtractGyroscope(agt_full_bits);
  imu.temp = ExtractTemperature(agt_full_bits);

  auto mag_full_bits = ReadMagnetometer();

  const auto mag_sensor_over_flow = mag_full_bits[6] & 0x08;  // HOFS 4th bit
  if (mag_sensor_over_flow)
  {
    printf("detect over flow\n");
  }
  else
  {
    imu.mag = ExtractMagnetometer(mag_full_bits);
  }

  return imu;
}

std::array<int16_t, 7> Mpu9250::ReadAccelGyroTemp()
{
  // Read raw data registers sequentially into data array
  const uint8_t num_reg = 14;
  uint8_t raw_data[num_reg];
  ReadMPURegisters(mpu9250::ACCEL_XOUT_H, num_reg, raw_data);

  // Turn the MSB and LSB into a signed 16-bit value
  std::array<int16_t, 7> full_bits{0};
  printf("full bits: ");
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    full_bits[i] = To16Bit(raw_data[i * 2], raw_data[i * 2 + 1]);
    printf("%d  ", full_bits[i]);
  }
  printf("\n");
  return full_bits;
}

Mpu9250::AccelData
Mpu9250::ExtractAccelerometer(const std::array<int16_t, 7>& full_bits)
{
  std::array<int16_t, 3> accel_full_bits{full_bits[0], full_bits[1],
                                         full_bits[2]};
  const auto data = ApplyPost(accel_full_bits, post_map[Accel]);
  AccelData accel;
  accel.data = ArrayToVec3(data);
  return accel;
}

Mpu9250::GyroData
Mpu9250::ExtractGyroscope(const std::array<int16_t, 7>& full_bits)
{
  std::array<int16_t, 3> gyro_full_bits{full_bits[4], full_bits[5],
                                        full_bits[6]};
  const auto data = ApplyPost(gyro_full_bits, post_map[Gyro]);
  GyroData gyro;
  gyro.data = ArrayToVec3(data);
  return gyro;
}

Mpu9250::TemperatureData
Mpu9250::ExtractTemperature(const std::array<int16_t, 7>& full_bits)
{
  const auto data = static_cast<float>(full_bits[3]);
  TemperatureData temp;
  temp.value = ((data - 21.F) / 333.87F) + 21.F;
  return temp;
}

std::array<int16_t, 3> Mpu9250::ReadMagnetometer()
{
  // RequestReadAK8963Registers(ak8963::XOUT_L, 7);
  // Read raw data registers sequentially into data array
  const uint8_t num_reg = 7;
  uint8_t raw_data[num_reg];
  ReadAK8963Registers(ak8963::XOUT_L, num_reg, raw_data);

  // Turn the MSB and LSB into a signed 16-bit value
  std::array<int16_t, 3> full_bits{0};
  printf("full bits: ");
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    full_bits[i] = To16Bit(raw_data[i * 2 + 1], raw_data[i * 2]);
    printf("%d  ", full_bits[i]);
  }
  printf("\n");

  return full_bits;
}

Mpu9250::MagData
Mpu9250::ExtractMagnetometer(const std::array<int16_t, 3>& full_bits)
{
  const auto data = ApplyPost(full_bits, post_map[Mag]);
  MagData mag;
  mag.data = ArrayToVec3(data);
  return mag;
}

void Mpu9250::Calibrate()
{
}

}  // namespace mpu