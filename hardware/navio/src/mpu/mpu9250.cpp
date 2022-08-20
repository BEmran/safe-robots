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

struct SpecInfo
{
  uint8_t byte;
  std::string name;
};


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
  // WriteRegister(mpu9250::PWR_MGMT_1, 0x80);
  // Delay(50);

  // Auto select clock source to be PLL gyroscope reference, else internal 20MHz
  WriteRegister(mpu9250::PWR_MGMT_1, 0x01);
  Delay(10);

  // Enable Acc & Gyro
  WriteRegister(mpu9250::PWR_MGMT_2, 0x00);
  Delay(10);

  // ConfigureI2C();
}

void Mpu9250::Initialize()
{
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  InitializeGyro();
  InitializeAccel();
  ConfigureI2C();
  InitializeMag();
  ExtractMagnetometerSensitivityAdjustmentValues();
  
  ReadAccelScaleAndBandWidth();
  ReadGyroScaleAndBandWidth();
  ReadMagModeAndResolution();
}

bool Mpu9250::Test()
{
  return false;
}

void Mpu9250::InitializeAccel() const
{
  // Set accelerometer full-scale range configuration
  uint8_t c1 = ReadRegister(mpu9250::ACCEL_CONFIG);
  c1 = static_cast<uint8_t>(c1 & 0xE7_uc);  // Clear AFS bits [4:3]
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
  const auto res = static_cast<uint8_t>(configured_scale|configured_mode);
  printf("scale %d mode %d or %d\n", configured_scale, configured_mode, res);
  WriteAK8963Register(ak8963::CNTL, res);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues()
{
  WriteAK8963Register(ak8963::CNTL, mag_mode_t::POWER_DOWN_MODE);
  Delay(10);
  WriteAK8963Register(ak8963::CNTL, mag_mode_t::FUSE_ROM_ACCESS_MODE);
  Delay(10);

  const auto asa_values = ReadAK8963Registers(ak8963::ASAX, 3);
  for (size_t i = 0; i < 3; i++)
  {
    sensitivity_calibration_[i] =
        static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;
    std::cout << sensitivity_calibration_[i] << std::endl;
  }

  // re-initialize magnetometer to reset mode
  InitializeMag();
}

void Mpu9250::ReadAccelScaleAndBandWidth() const
{
  const auto fs = ReadRegister(mpu9250::ACCEL_CONFIG) & 0x18;
   // Clear Fchoice bits [3] and AFS bits [2:0]
  const auto data = ReadRegister(mpu9250::ACCEL_CONFIG2);
  const auto f_choice_inv = data & 0x08;
  const auto bw = data & 0x7;

  switch (bw) {
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_5HZ)):
      std::cout << "Accel bandwidth: GBW_5HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_10HZ)):
      std::cout << "Accel bandwidth: GBW_10HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_21HZ)):
      std::cout << "Accel bandwidth: GBW_21HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_44HZ)):
      std::cout << "Accel bandwidth: GBW_44HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_99HZ)):
      std::cout << "Accel bandwidth: GBW_99HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelBandWidthHz::GBW_218HZ)):
      std::cout << "Accel bandwidth: GBW_218HZ" << std::endl;
      break;
    default:
      std::wcerr << "Accel bandwidth: undefined" << std::endl;
  }

  switch (f_choice_inv) {
    case (0x00):
      std::cout << "Accel: fChoice: enabled" << std::endl;
      break;
    default:
      std::wcerr << "Accel fChoice: disabled" << std::endl;
  }

  switch (fs) {
    case (static_cast<uint8_t>(AccelScale::AFS_2G)):
      std::cout << "Accel scale : AFS_2G" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelScale::AFS_4G)):
      std::cout << "Accel scale : AFS_4G" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelScale::AFS_8G)):
      std::cout << "Accel scale : AFS_8G" << std::endl;
      break;
    case (static_cast<uint8_t>(AccelScale::AFS_16G)):
      std::cout << "Accel scale : AFS_16G" << std::endl;
      break;
    default:
      std::wcerr << "Accel scale : undefined" << std::endl;
  }
}

void Mpu9250::ReadGyroScaleAndBandWidth() const
{
  const auto dlpf = ReadRegister(mpu9250::CONFIG) & 0x07;
   // Clear Fchoice bits [1:0] and AFS bits [4:3]
  const auto data = ReadRegister(mpu9250::GYRO_CONFIG);
  const auto f_choice_inv = data & 0x03;
  const auto fs = data & 0x18;

  switch (dlpf) {
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_5HZ)):
      std::cout << "Gyro bandwidth: GBW_5HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_10HZ)):
      std::cout << "Gyro bandwidth: GBW_10HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_20HZ)):
      std::cout << "Gyro bandwidth: GBW_20HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_41HZ)):
      std::cout << "Gyro bandwidth: GBW_41HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_92HZ)):
      std::cout << "Gyro bandwidth: GBW_92HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_184HZ)):
      std::cout << "Gyro bandwidth: GBW_184HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_250HZ)):
      std::cout << "Gyro bandwidth: GBW_250HZ" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroBandWidthHz::GBW_3600HZ)):
      std::cout << "Gyro bandwidth: GBW_3600HZ" << std::endl;
      break;
    default:
      std::wcerr << "Gyro bandwidth: undefined" << std::endl;
  }

  switch (f_choice_inv) {
    case (0x00):
      std::cout << "Gyro fChoice: enabled" << std::endl;
      break;
    default:
      std::wcerr << "Gyro fChoice: disabled" << std::endl;
  }

  switch (fs) {
    case (static_cast<uint8_t>(GyroScale::GFS_250DPS)):
      std::cout << "Gyro scale: GFS_250DPS" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroScale::GFS_500DPS)):
      std::cout << "Gyro scale: GFS_250DPS" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroScale::GFS_1000DPS)):
      std::cout << "Gyro scale: GFS_1000DPS" << std::endl;
      break;
    case (static_cast<uint8_t>(GyroScale::GFS_2000DPS)):
      std::cout << "Gyro scale: GFS_2000DPS" << std::endl;
      break;
    default:
      std::wcerr << "Gyro scale: undefined" << std::endl;
  }
}
void Mpu9250::ReadMagModeAndResolution() const
{
  const auto data = ReadAK8963Register(ak8963::CNTL);
  const auto mode = data & 0x0F;
  const auto res = data & 0x10;
  
  switch (mode) {
    case (static_cast<uint8_t>(MagMode::CONTINUES_8HZ_MODE)):
      std::cout << "Magnetometer mode: CONTINUES_8HZ_MODE" << std::endl;
      break;
    case (static_cast<uint8_t>(MagMode::CONTINUES_100HZ_MODE)):
      std::cout << "Magnetometer mode: CONTINUES_100HZ_MODE" << std::endl;
      break;
    case (static_cast<uint8_t>(mag_mode_t::POWER_DOWN_MODE)):
      std::cout << "Magnetometer mode: POWER_DOWN_MODE" << std::endl;
      break;
    case (static_cast<uint8_t>(mag_mode_t::FUSE_ROM_ACCESS_MODE)):
      std::cout << "Magnetometer mode: FUSE_ROM_ACCESS_MODE" << std::endl;
      break;
    case (static_cast<uint8_t>(mag_mode_t::SELF_TEST_MODE)):
      std::cout << "Magnetometer mode: SELF_TEST_MODE" << std::endl;
      break;
    case (static_cast<uint8_t>(mag_mode_t::SINGLE_MEASUREMENT_MODE)):
      std::cout << "Magnetometer mode: SINGLE_MEASUREMENT_MODE" << std::endl;
      break;
    default:
      std::wcerr << "Undefined Magnetometer mode" << std::endl;
  }

  switch (res) {
    case (static_cast<uint8_t>(MagScale::MFS_14BITS)):
      std::cout << "Magnetometer resolution: MFS_14BITS" << std::endl;
      break;
    case (static_cast<uint8_t>(MagScale::MFS_16BITS)):
      std::cout << "Magnetometer resolution: MFS_16BITS" << std::endl;
      break;
    default:
      std::wcerr << "Undefined Magnetometer resolution" << std::endl;
  }

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
  // Read raw data registers sequentially into data array
  const auto raw_data = ReadRegisters(mpu9250::ACCEL_XOUT_H, 21);
  const auto full_bits = ExtractFullBits(raw_data);

  ImuData imu;
  imu.accel = ExtractAccelerometer({full_bits[0], full_bits[1], full_bits[2]});
  imu.temp = ExtractTemperature(full_bits[3]);
  imu.gyro = ExtractGyroscope({full_bits[4], full_bits[5], full_bits[6]});
  const auto over_flow = raw_data[20] & 0x08;  // HOFS 4th bit
  imu.mag = ExtractMagnetometer({full_bits[7], full_bits[8], full_bits[9]}, over_flow);

  return imu;
}

std::vector<int16_t> Mpu9250::ExtractFullBits(const std::vector<uint8_t>& raw) const
{
  // Turn the MSB and LSB into a signed 16-bit value)
  std::vector<int16_t> full_bits(10, 0);
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    if (i<7)
      full_bits[i] = To16Bit(raw[i * 2], raw[i * 2 + 1]);
    else
      full_bits[i] = To16Bit(raw[i * 2 + 1], raw[i * 2]);
  }
  return full_bits;
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
  return ReadRegisters(reg, 1)[0];
}

std::vector<uint8_t> Mpu9250::ReadRegisters(const uint8_t reg, const uint8_t count)
{
  return GetSpi()->ReadRegisters(reg, count);
}

uint8_t Mpu9250::ReadAK8963Register(const uint8_t reg)
{
  return ReadAK8963Registers(reg, 1)[0];
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
  
  GetSpi()->WriteRegisters(reg_and_data);
  Delay(1);
}

std::vector<uint8_t> Mpu9250::ReadAK8963Registers(const uint8_t reg, const uint8_t count)
{
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  return GetSpi()->ReadRegisters(mpu9250::EXT_SENS_DATA_00, count);
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
    {mpu9250::I2C_SLV0_DO, data},
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
  // for (const auto rd : reg_and_data) 
  // {
    GetSpi()->WriteRegisters(reg_and_data);
  // }
}
}  // namespace mpu