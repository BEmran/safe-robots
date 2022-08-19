#include "mpu/mpu9250.hpp"
#include "mpu/spi.hpp"

namespace mpu
{
namespace
{

using SensorModuleType = core::sensors::SensorModuleType;
constexpr auto ImuType = SensorModuleType::IMU;
constexpr auto GyroType = SensorModuleType::GYROSCOPE;
constexpr auto MagType = SensorModuleType::MAGNETOMETER;
constexpr auto AccelType = SensorModuleType::ACCELEROMETER;

std::map<AccelScale, float> accel_scale_map = {{AccelScale::AFS_2G, 2.F},
                                               {AccelScale::AFS_4G, 4.F},
                                               {AccelScale::AFS_8G, 8.F},
                                               {AccelScale::AFS_16G, 16.F}};

std::map<GyroScale, float> gyro_scale_map = {{GyroScale::GFS_250DPS, 250.F},
                                             {GyroScale::GFS_500DPS, 500.F},
                                             {GyroScale::GFS_1000DPS, 1000.F},
                                             {GyroScale::GFS_2000DPS, 2000.F}};

std::map<MagScale, float> mag_scale_map = {
    {MagScale::MFS_14BITS, 0.25F * max_micro_tesla},
    {MagScale::MFS_16BITS, 1.00F * max_micro_tesla}};

}  // namespace

Mpu9250::Mpu9250(const Config& config, const bool debug)
  : ImuSensorModule(ImuType, SensorName, debug), config_(config)
{
  sensor_specs_map[AccelType] =
      CreateSensorSpecs(accel_scale_map[config.accel_scale], GRAVITY);
  sensor_specs_map[GyroType] =
      CreateSensorSpecs(gyro_scale_map[config.gyro_scale], PI / 180.0F);
  sensor_specs_map[MagType] =
      CreateSensorSpecs(mag_scale_map[config.mag_scale], 1.0F);
}

bool Mpu9250::ProbeMpu() const
{
  if (ReadRegister(mpu9250::WHO_AM_I) != mpu9250::WHO_AM_I_RESPONSE)
  {
    std::cout << "Bad IMU device ID" << std::endl;
    return false;
  }
  return true;
}

bool Mpu9250::ProbeAk8963() const
{
  ConfigureI2C();
  if (ReadAK8963Register(ak8963::WHO_AM_I) != ak8963::WHO_AM_I_RESPONSE)
  {
    std::cout << "Bad Magnetometer device ID" << std::endl;
    return false;
  }
  return true;
}

bool Mpu9250::Probe()
{
  if (ProbeMpu() && ProbeAk8963())
  {
    std::cout << "MPU9250 online!" << std::endl;
    return true;
  }
  return false;
}

void Mpu9250::ConfigureI2C()
{
  // enable master mode
  WriteRegister(mpu9250::USER_CTRL, mpu9250::I2C_MST_EN);

  // I2C configuration multi-master IIC 400KHz
  WriteRegister(mpu9250::I2C_MST_CTRL, 0x0D);
}

void Mpu9250::Reset()
{
  // wake up device, clear sleep mode bit (6)
  // WriteRegister(mpu9250::PWR_MGMT_1, 0x00);
  // Delay(100);

  // reset all registers to reset bit (7)
  WriteRegister(mpu9250::PWR_MGMT_1, 0x80);
  Delay(30);

  // Auto select clock source to be PLL gyroscope reference, else internal 20MHz
  WriteRegister(mpu9250::PWR_MGMT_1, 0x01);

  // Enable Acc & Gyro
  WriteRegister(mpu9250::PWR_MGMT_2, 0x00);

  ConfigureI2C();
}

void Mpu9250::Initialize()
{
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  InitializeAccel();
  InitializeGyro();
  InitializeMag();
}

bool Mpu9250::Test()
{
  return false;
}

void Mpu9250::InitializeAccel() const
{
  // Set accelerometer full-scale range configuration
  uint8_t c1 = ReadRegister(mpu9250::ACCEL_CONFIG);
  c1 = static_cast<uint8_t>(c1 & 0xF8_uc);  // Clear AFS bits [4:3]
  const uint8_t configured_scale = static_cast<uint8_t>(config_.accel_scale);
  c1 = static_cast<uint8_t>(c1 | configured_scale);
  // Write new ACCEL_CONFIG register value
  WriteRegister(mpu9250::ACCEL_CONFIG, c1);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.03
  // kHz
  // get current ACCEL_CONFIG2 register value
  uint8_t c2 = ReadRegister(mpu9250::ACCEL_CONFIG2);
  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c2 = static_cast<uint8_t>(c2 & 0xF0_uc);
  // Set accelerometer rate and bandwidth
  const uint8_t configured_bw = static_cast<uint8_t>(config_.accel_bw);
  c2 = static_cast<uint8_t>(c2 | configured_bw);
  // Write new ACCEL_CONFIG2 register value
  WriteRegister(mpu9250::ACCEL_CONFIG2, c2);

  Delay(10);
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
  WriteRegister(mpu9250::CONFIG, 0x01);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  // get current GYRO_CONFIG register value
  uint8_t c = ReadRegister(mpu9250::GYRO_CONFIG);
  // Clear Fchoice bits [1:0] and AFS bits [4:3]
  c = static_cast<uint8_t>(c & 0xE4_uc);
  const uint8_t configured_scale = static_cast<uint8_t>(config_.gyro_scale);
  c = static_cast<uint8_t>(c | configured_scale);
  // Write new GYRO_CONFIG value to register
  WriteRegister(mpu9250::GYRO_CONFIG, c);
  Delay(10);
}

void Mpu9250::InitializeMag() const
{
  // printf("write CNTL\n");
  // WriteAK8963Register(ak8963::CNTL, mag_mode_t::POWER_DOWN_MODE);
  // Delay(10);

  // Configure the magnetometer for continuous read and highest resolution
  // set Scale bit 4 to 1 or (0) to enable 16 or (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // Set magnetometer data resolution and sample ODR+
  const auto configured_scale = static_cast<uint8_t>(config_.mag_scale);
  const auto configured_mode = static_cast<uint8_t>(config_.mag_mode);
  WriteAK8963Register(ak8963::CNTL, configured_mode | configured_scale);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues()
{
  // printf("write CNTL\n");
  // WriteAK8963Register(ak8963::CNTL, mag_mode_t::POWER_DOWN_MODE);
  // Delay(10);
  // printf("write CNTL\n");
  // WriteAK8963Register(ak8963::CNTL, mag_mode_t::FUSE_ROM_ACCESS_MODE);
  // Delay(10);

  uint8_t asa_values[3];
  ReadAK8963Registers(ak8963::ASAX, 3, asa_values);
  for (size_t i = 0; i < 3; i++)
  {
    sensitivity_calibration_[i] =
        static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;
        std::cout << sensitivity_calibration_[i] << std::endl;
  }

  // re-initialize magnetometer to reset mode
  InitializeMag();
}

void Mpu9250::Calibrate()
{
  ExtractMagnetometerSensitivityAdjustmentValues();
}

void Mpu9250::Update()
{
  ImuData imu = ReadAll();
  SetData(imu);
}

ImuData Mpu9250::ReadAll() const
{
  RequestReadAK8963Registers(ak8963::XOUT_L, 7);
  Delay(10);

  ImuData imu = ReadAccelGyroTemp();
  imu.mag = ReadMagnetometer();

  return imu;
}

ImuData Mpu9250::ReadAccelGyroTemp() const
{
  // Read raw data registers sequentially into data array
  uint8_t raw_data[14];
  ReadRegisters(mpu9250::ACCEL_XOUT_H, 14, raw_data);

  // Turn the MSB and LSB into a signed 16-bit value)
  std::array<int16_t, 7> full_bits{0};
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    full_bits[i] = To16Bit(raw_data[i * 2], raw_data[i * 2 + 1]);
  }

  ImuData imu;
  imu.accel = ExtractAccelerometer({full_bits[0], full_bits[1], full_bits[2]});
  imu.temp = ExtractTemperature(full_bits[3]);
  imu.gyro = ExtractGyroscope({full_bits[4], full_bits[5], full_bits[6]});

  return imu;
}

Mpu9250::AccelData
Mpu9250::ExtractAccelerometer(const SensorFullBits& full_bits) const
{
  const auto data = ApplySensorSpecs(full_bits, sensor_specs_map[AccelType]);
  AccelData accel;
  accel.data = ArrayToVec3(data);
  return accel;
}

Mpu9250::GyroData
Mpu9250::ExtractGyroscope(const SensorFullBits& full_bits) const
{
  const auto data = ApplySensorSpecs(full_bits, sensor_specs_map[GyroType]);
  GyroData gyro;
  gyro.data = ArrayToVec3(data);
  return gyro;
}

Mpu9250::TemperatureData Mpu9250::ExtractTemperature(const int16_t full_bits)
{
  TemperatureData temp;
  temp.value = ((full_bits - 21.F) / 333.87F) + 21.F;
  return temp;
}

Mpu9250::MagData Mpu9250::ReadMagnetometer() const
{
  // Read raw data registers sequentially into data array
  uint8_t raw_data[7];
  ReadRegisters(mpu9250::EXT_SENS_DATA_00, 7, raw_data);
  // Turn the LSB and MSB into a signed 16-bit value
  SensorFullBits full_bits{0};
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    full_bits[i] = To16Bit(raw_data[i * 2 + 1], raw_data[i * 2]);
  }

  const auto over_flow = full_bits[6] & 0x08;  // HOFS 4th bit

  return ExtractMagnetometer(full_bits, over_flow);
}

Mpu9250::MagData Mpu9250::ExtractMagnetometer(const SensorFullBits& full_bits,
                                              const bool over_flow) const
{
  const auto data = ApplySensorSpecs(full_bits, sensor_specs_map[MagType]);
  MagData mag;
  if (over_flow)
  {
    std::wcerr << "detect over flow" << std::endl;
  }
  mag.data = ArrayToVec3(data);
  return mag;
}

uint8_t Mpu9250::ReadRegister(const uint8_t reg)
{
  uint8_t data[1] = {0};
  ReadRegisters(reg, 1, data);
  return data[0];
}

void Mpu9250::ReadRegisters(const uint8_t reg, const uint8_t count,
                            uint8_t* dest)
{
  GetSpi()->ReadRegisters(reg, count, dest);
}

uint8_t Mpu9250::ReadAK8963Register(const uint8_t reg)
{
  uint8_t data[1] = {0};
  ReadAK8963Registers(reg, 1, data);
  return data[0];
}

void Mpu9250::RequestReadAK8963Registers(const uint8_t reg, const uint8_t count)
{
  // // set slave 0 to the AK8963 and set for read
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_ADDR,
  //                         ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG);
  // // set the register to the desired AK8963 sub address
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_REG, reg);
  // // enable I2C and request the bytes
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
  // // takes some time for these registers to fill
  const std::vector<std::pair<uint8_t, uint8_t>> reg_and_data {
    {mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG},
    {mpu9250::I2C_SLV0_REG, reg},
    {mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count},
  };
  
  for (const auto rd : reg_and_data) 
  {
    GetSpi()->WriteRegister(rd.first, rd.second);
  }
  Delay(10);
}

void Mpu9250::ReadAK8963Registers(const uint8_t reg, const uint8_t count,
                                  uint8_t* dest)
{
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  GetSpi()->ReadRegisters(mpu9250::EXT_SENS_DATA_00, count, dest);
}

void Mpu9250::WriteRegister(const uint8_t reg, const uint8_t data)
{
  GetSpi()->WriteRegister(reg, data);
}

void Mpu9250::WriteAK8963Register(const uint8_t reg, const uint8_t data)
{
  constexpr uint8_t count = 1;
  const std::vector<std::pair<uint8_t, uint8_t>> reg_and_data {
    {mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR},
    {mpu9250::I2C_SLV0_REG, reg},
    {mpu9250::I2C_SLV0_DO, ak8963::I2C_ADDR},
    {mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count},
  };
  // // set slave 0 to the AK8963 and set for write
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR);
  // // set the register to the desired AK8963 sub address
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_REG, reg);
  // // store the data for write
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_DO, data);
  // // enable I2C and send 1 byte
  // GetSpi()->WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
  for (const auto rd : reg_and_data) 
  {
    GetSpi()->WriteRegister(rd.first, rd.second);
  }
}
}  // namespace mpu