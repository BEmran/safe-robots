#include "sensors/mpu/mpu9250.hpp"

#include "navio/hardware_utils.hpp"
#include "navio/spi.hpp"
#include "sensors/common/calibrate.hpp"

namespace sensors::mpu
{
namespace cu = common::utils;
using namespace cu::literals;  // _uc
namespace
{
using SensorModuleType = core::sensors::SensorModuleType;
constexpr auto ImuType = SensorModuleType::IMU;
constexpr auto GyroType = SensorModuleType::GYROSCOPE;
constexpr auto MagType = SensorModuleType::MAGNETOMETER;
constexpr auto AccelType = SensorModuleType::ACCELEROMETER;
constexpr auto TempType = SensorModuleType::TEMPERATURE;

cu::SensorSpecs CreateSensorSpecs(const cu::MATH_TYPE scale,
                                  const cu::MATH_TYPE unit)
{
  const auto sen = kMaxBitVal / scale;
  return cu::SensorSpecs(sen, unit);
}

std::string ConfigToString(const Config& cfg)
{
  std::stringstream ss;
  ss << "\tAccelerometer:"
     << " Bandwidth " << accel_bw_map[cfg.accel_bw].name
     << ", Full Scale: " << accel_scale_map[cfg.accel_scale].name
     << "\n\tGyroscope:"
     << " Bandwidth " << gyro_bw_map[cfg.gyro_bw].name
     << ", Full Scale: " << gyro_scale_map[cfg.gyro_scale].name
     << "\n\tMagnetometer:"
     << " Mode " << mag_mode_map[cfg.mag_mode].name
     << ", Full Scale: " << mag_scale_map[cfg.mag_scale].name
     << "\n\tSample rate devisor: "
     << static_cast<int>(cfg.sample_rate_divisor);
  return ss.str();
}

}  // namespace

Mpu9250::Mpu9250(const Config& config, std::unique_ptr<navio::SPI> comm,
                 std::unique_ptr<core::utils::Node> node)
  : cu::ImuSensorModule(ImuType, SensorName, true)
  , config_(config)
  , comm_{std::move(comm)}
  , node_{std::move(node)}

{
  sensor_specs_map[AccelType] =
      CreateSensorSpecs(accel_scale_map[config.accel_scale].value, cu::GRAVITY);
  sensor_specs_map[GyroType] = CreateSensorSpecs(
      gyro_scale_map[config.gyro_scale].value, cu::PI / 180.0F);
  sensor_specs_map[MagType] =
      CreateSensorSpecs(mag_scale_map[config.mag_scale].value, 1.0F);
  sensor_specs_map[TempType] =
      cu::SensorSpecs(TempScale, 1.0F, kTempBias, kTempOffset);

  cu::Mat3 accel_misalignment;
  accel_misalignment << 0.998122F, 0.00794836F, 0.000548448F,  //
      -0.00552448F, 0.998181F, -0.00669443F,                   //
      0.0189156F, 0.00407755F, 0.993244F;
  cu::Vec3 accel_bias{-0.00387028F, -0.0128085F, 0.0108167F};
  cu::Vec3 gyro_bias{12.629F, 7.572F, -9.618F};
  sensor_specs_map[AccelType].SetCalibration(accel_misalignment, accel_bias,
                                             cu::Vec3::Zero());  //
  sensor_specs_map[GyroType].SetCalibration(cu::Mat3::Identity(), gyro_bias,
                                            cu::Vec3::Zero());  //
}

bool Mpu9250::ProbeMpu() const
{
  if (ReadRegister(mpu9250::WHO_AM_I) != mpu9250::WHO_AM_I_RESPONSE)
  {
    node_->LogWarn("Bad IMU device ID");
    return false;
  }
  return true;
}

bool Mpu9250::ProbeAk8963() const
{
  ConfigureI2C();
  if (ReadAK8963Register(ak8963::WHO_AM_I) != ak8963::WHO_AM_I_RESPONSE)
  {
    node_->LogWarn("Bad Magnetometer device ID");
    return false;
  }
  return true;
}

bool Mpu9250::Probe()
{
  if (ProbeMpu() && ProbeAk8963())
  {
    node_->LogDebug("MPU9250 is online!");
    return true;
  }
  return false;
}

void Mpu9250::ConfigureI2C() const
{
  // enable master mode
  WriteRegister(mpu9250::USER_CTRL, mpu9250::I2C_MST_EN);

  // I2C configuration multi-master IIC 400KHz
  WriteRegister(mpu9250::I2C_MST_CTRL, 0x0D);
}

void Mpu9250::Reset() const
{
  // reset all registers to reset bit (7)
  WriteRegister(mpu9250::PWR_MGMT_1, 0x80);
  navio::hardware_utils::Delay(100);

  // Auto select clock source to be PLL gyroscope reference, else internal
  // 20MHz
  WriteRegister(mpu9250::PWR_MGMT_1, 0x01);
  navio::hardware_utils::Delay(10);

  // Enable Acc & Gyro
  WriteRegister(mpu9250::PWR_MGMT_2, 0x00);
  navio::hardware_utils::Delay(10);

  // ConfigureI2C();
}

void Mpu9250::Initialize()
{
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  // disable fifo
  WriteRegister(mpu9250::FIFO_EN, 0x00);

  InitializeGyro();
  InitializeAccel();
  ConfigureI2C();
  InitializeMag();

  ValidateConfiguration();
}

bool Mpu9250::Test()
{
  return false;
}

void Mpu9250::InitializeAccel() const
{
  // Set accelerometer full-scale range configuration
  const auto scale = accel_scale_map[config_.accel_scale].byte;
  const auto mask1 = 0xE7_uc;
  SetRegisterByte(mpu9250::ACCEL_CONFIG, scale, mask1);

  // Set accelerometer sample rate configuration
  const auto bw = accel_bw_map[config_.accel_bw].byte;
  const auto mask2 = 0xF0_uc;
  SetRegisterByte(mpu9250::ACCEL_CONFIG2, bw, mask2);
}

void Mpu9250::InitializeGyro() const
{
  // Configure Gyro and Thermometer bandwidth
  const auto bw = gyro_bw_map[config_.gyro_bw].byte;
  const auto cmask = 0xF8_uc;
  SetRegisterByte(mpu9250::CONFIG, bw, cmask);

  // Set gyroscope full scale range
  const auto scale = gyro_scale_map[config_.gyro_scale].byte;
  const auto gmask = 0xE4_uc;
  SetRegisterByte(mpu9250::GYRO_CONFIG, scale, gmask);
}

void Mpu9250::InitializeMag() const
{
  const auto scale = mag_scale_map[config_.mag_scale].byte;
  const auto mode = mag_mode_map[config_.mag_mode].byte;
  node_->LogInfo(std::to_string(mode));
  const auto res = static_cast<uint8_t>(scale | mode);
  WriteAK8963Register(ak8963::CNTL, res);
  navio::hardware_utils::Delay(10);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues()
{
  WriteAK8963Register(ak8963::CNTL, mag_mode_map[MagMode::POWER_DOWN].byte);
  navio::hardware_utils::Delay(10);

  WriteAK8963Register(ak8963::CNTL,
                      mag_mode_map[MagMode::FUSE_ROM_ACCESS].byte);
  navio::hardware_utils::Delay(10);

  const auto asa_values = ReadAK8963Registers(ak8963::ASAX, 3);
  for (size_t i = 0; i < 3; i++)
  {
    mag_sensitivity_calibration_[i] =
        static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;
    std::cout << mag_sensitivity_calibration_[i] << std::endl;
  }

  // re-initialize magnetometer to reset mode
  InitializeMag();
}

AccelScale Mpu9250::ReadAccelScale() const
{
  const auto data = ReadRegister(mpu9250::ACCEL_CONFIG);
  const auto fs = static_cast<uint8_t>(data & 0x18);  // mask bits [4:3]
  return accel_scale_map.Find(fs);
}

AccelBandWidthHz Mpu9250::ReadAccelBandWidth() const
{
  const auto data = ReadRegister(mpu9250::ACCEL_CONFIG2);
  const auto f_inv = static_cast<uint8_t>(data & 0x08);  // mask bits [3]
  const auto bw = static_cast<uint8_t>(data & 0x07);     // mask bits [2:0]
  if (f_inv != 0)
  {
    node_->LogError("Accelerometer fChoice is disabled");
  }
  return accel_bw_map.Find(bw);
}

GyroBandWidthHz Mpu9250::ReadGyroBandWidth() const
{
  const auto data = ReadRegister(mpu9250::CONFIG);
  const auto bw = static_cast<uint8_t>(data & 0x07);  // mask bits [2:0]
  return gyro_bw_map.Find(bw);
}

GyroScale Mpu9250::ReadGyroScale() const
{
  const auto data = ReadRegister(mpu9250::GYRO_CONFIG);
  const auto f_inv = static_cast<uint8_t>(data & 0x03);  // mask bits [1:0]
  const auto fs = static_cast<uint8_t>(data & 0x18);     // mask bits [4:3]
  if (f_inv != 0)
  {
    node_->LogError("Gyroscope fChoice is disabled");
  }
  return gyro_scale_map.Find(fs);
}

MagMode Mpu9250::ReadMagMode() const
{
  const auto data = ReadAK8963Register(ak8963::CNTL);
  const auto mode = static_cast<uint8_t>(data & 0x0F);  // mask bits[3:0]
  return mag_mode_map.Find(mode);
}

MagScale Mpu9250::ReadMagScale() const
{
  const auto data = ReadAK8963Register(ak8963::CNTL);
  const auto res = static_cast<uint8_t>(data & 0x10);  // mask res bit[4]
  return mag_scale_map.Find(res);
}

std::pair<bool, Config> Mpu9250::ValidateConfiguration() const
{
  Config actual_cfg;
  actual_cfg.accel_bw = ReadAccelBandWidth();
  actual_cfg.accel_scale = ReadAccelScale();
  actual_cfg.gyro_bw = ReadGyroBandWidth();
  actual_cfg.gyro_scale = ReadGyroScale();
  actual_cfg.mag_mode = ReadMagMode();
  actual_cfg.mag_scale = ReadMagScale();
  actual_cfg.sample_rate_divisor = ReadRegister(mpu9250::SMPLRT_DIV);

  bool valid = true;
  valid &= actual_cfg.accel_bw == config_.accel_bw;
  valid &= actual_cfg.accel_scale == config_.accel_scale;
  valid &= actual_cfg.gyro_bw == config_.gyro_bw;
  valid &= actual_cfg.gyro_scale == config_.gyro_scale;
  valid &= actual_cfg.mag_mode == config_.mag_mode;
  valid &= actual_cfg.mag_scale == config_.mag_scale;
  valid &= actual_cfg.sample_rate_divisor == config_.sample_rate_divisor;

  const auto config_str = ConfigToString(actual_cfg);
  const std::string valid_str = valid ? "successfully" : "failed to";
  const std::string msg = "MPU9250 " + valid_str +
                          " initialized! Actual configuration is set to:\n" +
                          config_str;
  if (valid)
  {
    node_->LogDebug(msg);
  }
  else
  {
    node_->LogWarn(msg);
  }
  return {valid, actual_cfg};
}

void Mpu9250::Calibrate()
{
  // ExtractMagnetometerSensitivityAdjustmentValues();
  auto read_gyro_data = [this]() { return ReadRawData().gyro; };
  auto read_accel_data = [this]() { return ReadRawData().accel; };
  auto read_mag_data = [this]() { return ReadRawData().mag; };

  // sensor_specs_map[GyroType] =
  //     common::calibrate::CalibrateGyroscope(read_gyro_data,
  //     sensor_specs_map[GyroType]);
  // sensor_specs_map[AccelType] =
  //     common::calibrate::CalibrateAccelerometer(read_accel_data,
  //     sensor_specs_map[AccelType]);
  sensor_specs_map[MagType] = common::calibrate::CalibrateMagnetometer(
      read_mag_data, sensor_specs_map[MagType]);
}

void Mpu9250::Update()
{
  const auto raw = ReadRawData();
  cu::ImuData imu = ApplySensorSpecs(raw);
  imu.tait_bryan.data = cu::EstimateRPY(imu.accel.data);
  SetData(imu);
}

SensorRawData Mpu9250::ReadRawData() const
{
  RequestReadAK8963Registers(ak8963::XOUT_L, 7);
  const auto data = ReadRegisters(mpu9250::ACCEL_XOUT_H, 21);
  const auto full_bits = ExtractFullBits(data);
  return FullBitsToRawData(full_bits);
}

std::vector<int16_t> Mpu9250::ExtractFullBits(const std::vector<uint8_t>& data)
{
  assert(data.size() == 21);
  // Turn the MSB and LSB into a signed 16-bit value)
  std::vector<int16_t> full_bits(11, 0);
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    if (i < 7)
    {
      full_bits[i] = cu::To16Bit(data[i * 2], data[i * 2 + 1]);
    }
    else
    {
      full_bits[i] = cu::To16Bit(data[i * 2 + 1], data[i * 2]);
    }
  }
  full_bits[10] = data[20];
  return full_bits;
}

SensorRawData Mpu9250::FullBitsToRawData(const std::vector<int16_t>& full_bits)
{
  assert(full_bits.size() == 11);
  SensorRawData raw;
  raw.accel = cu::Vec3From16BitsVector(full_bits.begin() + 0);
  raw.gyro = cu::Vec3From16BitsVector(full_bits.begin() + 4);
  raw.mag = cu::Vec3From16BitsVector(full_bits.begin() + 7);
  raw.temp = static_cast<float>(full_bits[3]);
  raw.mag_over_flow = full_bits[10] == 0x08_uc;  // HOFS 4th bit
  return raw;
}

cu::ImuData Mpu9250::ApplySensorSpecs(const SensorRawData& raw) const
{
  cu::ImuData imu;
  imu.accel.data = sensor_specs_map[AccelType].Apply(raw.accel);
  imu.gyro.data = sensor_specs_map[GyroType].Apply(raw.gyro);
  imu.mag.data = sensor_specs_map[MagType].Apply(raw.mag);
  imu.temp.value = sensor_specs_map[TempType].Apply(raw.temp);
  if (raw.mag_over_flow)
  {
    node_->LogDebug("detect over flow");
    // imu.mag.data = cu::Vec3::Zero();
  }
  return imu;
}

uint8_t Mpu9250::ReadRegister(const uint8_t reg) const
{
  return ReadRegisters(reg, 1)[0];
}

std::vector<uint8_t> Mpu9250::ReadRegisters(const uint8_t reg,
                                            const uint8_t count) const
{
  return comm_->ReadRegisters(reg, count);
}

uint8_t Mpu9250::ReadAK8963Register(const uint8_t reg) const
{
  return ReadAK8963Registers(reg, 1)[0];
}

void Mpu9250::RequestReadAK8963Registers(const uint8_t reg,
                                         const uint8_t count) const
{
  const std::vector<std::pair<uint8_t, uint8_t>> reg_and_data{
      {mpu9250::I2C_SLV0_ADDR,
       ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG},  // set slave 0 to the
                                                    // AK8963 and set for read
      {mpu9250::I2C_SLV0_REG, reg},  // set the register to the desired AK8963
                                     // sub address
      {mpu9250::I2C_SLV0_CTRL,
       mpu9250::I2C_SLV0_EN | count},  // enable I2C and request the bytes
  };

  comm_->WriteRegisters(reg_and_data);
  navio::hardware_utils::Delay(1);
}

std::vector<uint8_t> Mpu9250::ReadAK8963Registers(const uint8_t reg,
                                                  const uint8_t count) const
{
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  return comm_->ReadRegisters(mpu9250::EXT_SENS_DATA_00, count);
}

void Mpu9250::SetRegisterByte(const uint8_t reg, const uint8_t byte,
                              const uint8_t mask) const
{
  const auto current = ReadRegister(reg);
  WriteRegister(reg, cu::SetFlags(current, mask, byte));
}

void Mpu9250::WriteRegister(const uint8_t reg, const uint8_t data) const
{
  comm_->WriteRegister(reg, data);
}

void Mpu9250::WriteAK8963Register(const uint8_t reg, const uint8_t data) const
{
  constexpr uint8_t count = 1;
  const std::vector<std::pair<uint8_t, uint8_t>> reg_and_data{
      {mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR},  // set slave 0 to the
                                                   // AK8963 and set for write
      {mpu9250::I2C_SLV0_REG, reg},  // set the register to the desired AK8963
                                     // sub address
      {mpu9250::I2C_SLV0_DO, data},  // store the data for write
      {mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count},  // enable I2C
                                                               // and send 1
                                                               // byte
  };
  comm_->WriteRegisters(reg_and_data);
}
}  // namespace sensors::mpu