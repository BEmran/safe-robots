// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/sensors/mpu/mpu9250.hpp"

#include "common/sensors/calibrate.hpp"
#include "common/utils.hpp"

namespace hardware::common::sensors::mpu {
// namespace cu = common::utils;

namespace {
using SensorModuleType = core::sensors::SensorModuleType;
constexpr auto ImuType = SensorModuleType::IMU;
// constexpr auto GyroType = SensorModuleType::GYROSCOPE;
// constexpr auto MagType = SensorModuleType::MAGNETOMETER;
// constexpr auto AccelType = SensorModuleType::ACCELEROMETER;
// constexpr auto TempType = SensorModuleType::TEMPERATURE;
constexpr auto ShortDelay = 10;
constexpr auto LongDelay = 100;

/**
 * @brief Construct a new Sensor Specs object using scale and uint info
 *
 * @param scale measurement scale
 * @param unit unit conversion row -> iso unit
 */
SensorSpecs<3> CreateSensorSpecs(const MATH_TYPE scale, const MATH_TYPE unit) {
  const auto sen = kMaxBitVal / scale;
  return SensorSpecs<3>(sen, unit);
}

std::string ConfigToString(const Config& cfg) {
  std::stringstream ss;
  ss << "\tAccelerometer:"
     << " Bandwidth " << AccelBWMap()[cfg.accel_bw]
     << ", Full Scale: " << AccelScaleMap()[cfg.accel_scale].name
     << "\n\tGyroscope:"
     << " Bandwidth " << GyroBWMap()[cfg.gyro_bw]
     << ", Full Scale: " << GyroScaleMap()[cfg.gyro_scale].name
     << "\n\tMagnetometer:"
     << " Mode " << MagModeMap()[cfg.mag_mode]
     << ", Full Scale: " << MagScaleMap()[cfg.mag_scale].name
     << "\n\tSample rate devisor: "
     << static_cast<int>(cfg.sample_rate_divisor);
  return ss.str();
}

}  // namespace

Mpu9250::Mpu9250(const Config& config,
                 std::unique_ptr<comm::CommunicationAbs> comm,
                 std::unique_ptr<core::utils::Node> node)
  : ImuSensorModule(ImuType, SensorName, true)
  , config_(config)
  , comm_{std::move(comm)}
  , node_{std::move(node)}
  , mag_spec_{CreateSensorSpecs(MagScaleMap()[config.mag_scale].value, 1.0F)}
  , gyro_spec_{CreateSensorSpecs(GyroScaleMap()[config.gyro_scale].value,
                                 DEG_TO_RAD)}
  , accel_spec_{CreateSensorSpecs(AccelScaleMap()[config.accel_scale].value,
                                  GRAVITY)}
  , temp_spec_{SensorSpecs<1>(TempSensitivity, 1.0F)} {
  temp_spec_.SetCalibration(CreateScalar(1), CreateScalar(kTempBias),
                            CreateScalar(kTempOffset));
  ReadCalibrationFile();
}

void Mpu9250::ReadCalibrationFile() {
  // TODO(Bara) read from config file instead
  Mat3 accel_misalignment;

  accel_misalignment << (Mat3() << 0.998122F, 0.00794836F, 0.000548448F,
                         -0.00552448F, 0.998181F, -0.00669443F, 0.0189156F,
                         0.00407755F, 0.993244F)
                          .finished();
  Vec3 accel_bias(-0.00387028F, -0.0128085F, 0.0108167F);
  Vec3 gyro_bias(12.629F, 7.572F, -9.618F);

  accel_spec_.SetCalibration(accel_misalignment, accel_bias,
                             Vec3::Zero());  //
  gyro_spec_.SetCalibration(Mat3::Identity(), gyro_bias,
                            Vec3::Zero());  //
}

bool Mpu9250::ProbeMpu() const {
  if (ReadRegister(mpu9250::WHO_AM_I) != mpu9250::WHO_AM_I_RESPONSE) {
    node_->GetNodeLogger()->Warn("Bad IMU device ID");
    return false;
  }
  return true;
}

bool Mpu9250::ProbeAk8963() const {
  ConfigureI2C();
  if (ReadAK8963Register(ak8963::WHO_AM_I) != ak8963::WHO_AM_I_RESPONSE) {
    node_->GetNodeLogger()->Warn("Bad Magnetometer device ID");
    return false;
  }
  return true;
}

bool Mpu9250::Probe() {
  if (ProbeMpu() && ProbeAk8963()) {
    node_->GetNodeLogger()->Debug("MPU9250 is online!");
    return true;
  }
  return false;
}

void Mpu9250::ConfigureI2C() const {
  // enable master mode
  WriteRegister(mpu9250::USER_CTRL, mpu9250::I2C_MST_EN);

  // I2C configuration multi-master IIC 400KHz
  constexpr auto i2c_clock = 0x0D;
  WriteRegister(mpu9250::I2C_MST_CTRL, i2c_clock);
}

void Mpu9250::Reset() const {
  // reset all registers to reset bit (7)
  constexpr auto reset_byte = 0x80;
  WriteRegister(mpu9250::PWR_MGMT_1, reset_byte);
  MilliDelay(LongDelay);

  // Auto select clock source to be PLL gyroscope, else internal 20MHz
  constexpr auto auto_select_clock_source = 0x01;
  WriteRegister(mpu9250::PWR_MGMT_1, auto_select_clock_source);
  MilliDelay(ShortDelay);

  // Enable Acc & Gyro
  constexpr auto enable_byte = 0x00;
  WriteRegister(mpu9250::PWR_MGMT_2, enable_byte);
  MilliDelay(ShortDelay);
}

void Mpu9250::Initialize() {
  Reset();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  // disable fifo
  constexpr auto disable = 0x00;
  WriteRegister(mpu9250::FIFO_EN, disable);

  InitializeGyro();
  InitializeAccel();
  ConfigureI2C();
  InitializeMag();

  ValidateConfiguration();
}

bool Mpu9250::Test() {
  return false;
}

void Mpu9250::InitializeAccel() const {
  // Set accelerometer full-scale range configuration
  const auto scale = ToByte(config_.accel_scale);
  constexpr auto mask1 = 0xE7;
  SetRegisterByte(mpu9250::ACCEL_CONFIG, scale, mask1);

  // Set accelerometer sample rate configuration
  const auto bw = ToByte(config_.accel_bw);
  constexpr auto mask2 = 0xF0;
  SetRegisterByte(mpu9250::ACCEL_CONFIG2, bw, mask2);
}

void Mpu9250::InitializeGyro() const {
  // Configure Gyro and Thermometer bandwidth
  const auto bw = ToByte(config_.gyro_bw);
  constexpr auto cmask = 0xF8;
  SetRegisterByte(mpu9250::CONFIG, bw, cmask);

  // Set gyroscope full scale range
  const auto scale = ToByte(config_.gyro_scale);
  constexpr auto gmask = 0xE4;
  SetRegisterByte(mpu9250::GYRO_CONFIG, scale, gmask);
}

void Mpu9250::InitializeMag() const {
  const auto scale = ToByte(config_.mag_scale);
  const auto mode = ToByte(config_.mag_mode);
  node_->GetNodeLogger()->Info(std::to_string(mode));
  const auto res = static_cast<uint8_t>(scale | mode);
  WriteAK8963Register(ak8963::CNTL, res);
  MilliDelay(ShortDelay);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues() {
  WriteAK8963Register(ak8963::CNTL, ToByte(MagMode::POWER_DOWN));
  MilliDelay(ShortDelay);

  WriteAK8963Register(ak8963::CNTL, ToByte(MagMode::FUSE_ROM_ACCESS));
  MilliDelay(ShortDelay);

  constexpr auto reg_size = 3;
  const auto asa_values = ReadAK8963Registers(ak8963::ASAX, reg_size);
  for (size_t i = 0; i < asa_values.size(); i++) {
    const long idx = static_cast<long>(i);
    mag_sensitivity_calibration_[idx] =
      static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;  // NOLINT
  }
  std::stringstream msg;
  msg << "Magnetometer Sensitivity Adjustment Values: "
      << mag_sensitivity_calibration_.transpose();
  node_->GetNodeLogger()->Debug(msg.str());

  // re-initialize magnetometer to reset mode
  InitializeMag();
}

AccelScale Mpu9250::ReadAccelScale() const {
  const auto data = ReadRegister(mpu9250::ACCEL_CONFIG);
  constexpr auto mask = 0x18;  // mask bits [4:3]
  const auto fs = static_cast<uint8_t>(data & mask);
  return Find(AccelScaleMap(), fs);
}

AccelBandWidthHz Mpu9250::ReadAccelBandWidth() const {
  const auto data = ReadRegister(mpu9250::ACCEL_CONFIG2);
  constexpr auto mask3_f_inv = 0x08;  // mask bits [3]
  const auto f_inv = static_cast<uint8_t>(data & mask3_f_inv);
  constexpr auto mask_bw = 0x07;  // mask bits [2:0]
  const auto bw = static_cast<uint8_t>(data & mask_bw);
  if (f_inv != 0) {
    node_->GetNodeLogger()->Error("Accelerometer fChoice is disabled");
  }
  return Find(AccelBWMap(), bw);
}

GyroBandWidthHz Mpu9250::ReadGyroBandWidth() const {
  const auto data = ReadRegister(mpu9250::CONFIG);
  constexpr auto mask_bw = 0x07;  // mask bits [2:0]
  const auto bw = static_cast<uint8_t>(data & mask_bw);
  return Find(GyroBWMap(), bw);
}

GyroScale Mpu9250::ReadGyroScale() const {
  const auto data = ReadRegister(mpu9250::GYRO_CONFIG);
  constexpr auto mask3_f_inv = 0x03;  // mask bits [1:0]
  const auto f_inv = static_cast<uint8_t>(data & mask3_f_inv);
  constexpr auto mask_fs = 0x18;  // mask bits [4:3]
  const auto fs = static_cast<uint8_t>(data & mask_fs);
  if (f_inv != 0) {
    node_->GetNodeLogger()->Error("Gyroscope fChoice is disabled");
  }
  return Find(GyroScaleMap(), fs);
}

MagMode Mpu9250::ReadMagMode() const {
  const auto data = ReadAK8963Register(ak8963::CNTL);
  constexpr auto mask = 0x0F;  // mask bits[3:0]
  const auto mode = static_cast<uint8_t>(data & mask);
  return Find(MagModeMap(), mode);
}

MagScale Mpu9250::ReadMagScale() const {
  const auto data = ReadAK8963Register(ak8963::CNTL);
  constexpr auto mask = 0x10;  // mask res bit[4]
  const auto res = static_cast<uint8_t>(data & mask);
  return Find(MagScaleMap(), res);
}

std::pair<bool, Config> Mpu9250::ValidateConfiguration() const {
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
  if (valid) {
    node_->GetNodeLogger()->Debug(msg);
  } else {
    node_->GetNodeLogger()->Warn(msg);
  }
  return {valid, actual_cfg};
}

void Mpu9250::Calibrate() {
  ExtractMagnetometerSensitivityAdjustmentValues();
  auto read_gyro_data = [this]() { return ReadRawData().gyro; };
  auto read_accel_data = [this]() { return ReadRawData().accel; };
  auto read_mag_data = [this]() { return ReadRawData().mag; };

  gyro_spec_ = CalibrateGyroscope(read_gyro_data, gyro_spec_);
  accel_spec_ = CalibrateAccelerometer(read_accel_data, accel_spec_);
  mag_spec_ = CalibrateMagnetometer(read_mag_data, mag_spec_);
}

void Mpu9250::Update() {
  const auto raw = ReadRawData();
  core::utils::ImuData imu = ApplySensorSpecs(raw);
  // imu.tait_bryan.data = EstimateRPY(imu.accel.data);
  SetData(imu);
}

SensorRawData Mpu9250::ReadRawData() const {
  constexpr auto mag_data_size = 7;
  RequestReadAK8963Registers(ak8963::XOUT_L, mag_data_size);
  constexpr auto mpu_data_size = 21;
  const auto data = ReadRegisters(mpu9250::ACCEL_XOUT_H, mpu_data_size);
  const auto full_bits = ExtractFullBits(data);
  return FullBitsToRawData(full_bits);
}

std::vector<int16_t>
Mpu9250::ExtractFullBits(const std::vector<uint8_t>& data) {
  constexpr auto mpu_data_size = 21;
  assert(data.size() == mpu_data_size);
  // Turn the MSB and LSB into a signed 16-bit value)
  constexpr auto full_bits_size = 11;
  std::vector<int16_t> full_bits(full_bits_size, 0);
  constexpr auto last_mpu_full_bits_idx = 7;
  for (size_t i = 0; i < full_bits.size() - 1; i++) {
    if (i < last_mpu_full_bits_idx) {
      full_bits[i] = To16Bit(data[i * 2], data[i * 2 + 1]);
    } else {
      full_bits[i] = To16Bit(data[i * 2 + 1], data[i * 2]);
    }
  }
  constexpr auto over_flow_bits_idx = 10;
  constexpr auto over_data_idx = 20;
  full_bits[over_flow_bits_idx] = data[over_data_idx];
  return full_bits;
}

SensorRawData
Mpu9250::FullBitsToRawData(const std::vector<int16_t>& full_bits) {
  assert(full_bits.size() == 11);
  SensorRawData raw;
  constexpr auto accel_start_idx = 0;
  constexpr auto gyro_start_idx = 4;
  constexpr auto mag_start_idx = 7;
  raw.accel = Vec3From16BitsVector(full_bits.begin() + accel_start_idx);
  raw.gyro = Vec3From16BitsVector(full_bits.begin() + gyro_start_idx);
  raw.mag = Vec3From16BitsVector(full_bits.begin() + mag_start_idx);
  constexpr auto temp_idx = 3;
  raw.temp = static_cast<float>(full_bits[temp_idx]);
  constexpr auto mask = 0x08;  // HOFS 4th bit
  constexpr auto over_flow_idx = 10;
  raw.mag_over_flow = full_bits[over_flow_idx] == mask;
  return raw;
}

ImuData Mpu9250::ApplySensorSpecs(const SensorRawData& raw) const {
  ImuData imu;
  imu.accel.data = accel_spec_.Apply(raw.accel);
  imu.gyro.data = gyro_spec_.Apply(raw.gyro);
  imu.mag.data = mag_spec_.Apply(raw.mag);
  imu.temp.value = static_cast<double>(temp_spec_.Apply(raw.temp));
  if (raw.mag_over_flow) {
    node_->GetNodeLogger()->Debug("detect over flow");
    // imu.mag.data = Vec3::Zero();
  }
  return imu;
}

uint8_t Mpu9250::ReadRegister(uint8_t reg) const {
  return ReadRegisters(reg, 1)[0];
}

std::vector<uint8_t> Mpu9250::ReadRegisters(uint8_t reg, uint8_t count) const {
  return comm_->ReadBytes(reg, count);
}

uint8_t Mpu9250::ReadAK8963Register(uint8_t reg) const {
  return ReadAK8963Registers(reg, 1)[0];
}

void Mpu9250::RequestReadAK8963Registers(uint8_t reg, uint8_t count) const {
  // set slave 0 to the AK8963 and set for read
  constexpr uint8_t read_address = ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG;
  comm_->WriteByte(mpu9250::I2C_SLV0_ADDR, read_address);
  // set the register to the desired AK8963 sub address
  comm_->WriteByte(mpu9250::I2C_SLV0_REG, reg);
  // enable I2C and request the bytes
  comm_->WriteByte(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
  MilliDelay(1);
}

std::vector<uint8_t> Mpu9250::ReadAK8963Registers(uint8_t reg,
                                                  uint8_t count) const {
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  return comm_->ReadBytes(mpu9250::EXT_SENS_DATA_00, count);
}

void Mpu9250::SetRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const {
  const auto current = ReadRegister(reg);
  WriteRegister(reg, SetFlags(current, mask, byte));
}

void Mpu9250::WriteRegister(uint8_t reg, uint8_t data) const {
  comm_->WriteByte(reg, data);
}

void Mpu9250::WriteAK8963Register(uint8_t reg, uint8_t data) const {
  // set slave 0 to the AK8963 and set for write
  comm_->WriteByte(mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR);
  // set the register to the desired AK8963 sub address
  comm_->WriteByte(mpu9250::I2C_SLV0_REG, reg);
  // store the data for write
  comm_->WriteByte(mpu9250::I2C_SLV0_DO, data);
  // enable I2C and send 1 byte
  constexpr uint8_t count = 1;
  comm_->WriteByte(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
}
}  // namespace hardware::common::sensors::mpu
