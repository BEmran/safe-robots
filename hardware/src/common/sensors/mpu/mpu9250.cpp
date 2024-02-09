// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "common/sensors/mpu/mpu9250.hpp"

#include "common/sensors/calibrate.hpp"
#include "common/utils.hpp"
#include "core/utils/logger_macros.hpp"

namespace hardware::common::sensors::mpu {

namespace {
using SensorModuleType = core::sensors::SensorModuleType;
constexpr auto ImuType = SensorModuleType::IMU;
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

template <typename Config>
Config ConfigValueOr(std::string_view config_name,
                     const std::optional<Config> config,
                     const Config default_value) {
  std::cout << "Set Configuration for " << config_name.data();
  if (not config.has_value()) {
    std::cout << " is undefined, use default value instead: ";
  } else {
    std::cout << " use set to value: ";
  }
  std::cout << std::hex << ToByte(config.value_or(default_value)) << std::endl;
  return config.value_or(default_value);
}

std::string ConfigToString(const Config& cfg) {
  std::stringstream ss;
  ss << "\tAccelerometer:"
     << " Bandwidth " << AccelBWConfigMap.Name(cfg.accel_bw)
     << ", Full Scale: " << AccelScaleConfigMap.Name(cfg.accel_scale)
     << "\n\tGyroscope:"
     << " Bandwidth " << GyroBWConfigMap.Name(cfg.gyro_bw)
     << ", Full Scale: " << GyroScaleConfigMap.Name(cfg.gyro_scale)
     << "\n\tMagnetometer:"
     << " Mode " << MagModeConfigMap.Name(cfg.mag_mode)
     << ", Full Scale: " << MagScaleConfigMap.Name(cfg.mag_scale)
     << "\n\tSample rate devisor: "
     << static_cast<int>(cfg.sample_rate_divisor);
  return ss.str();
}

core::utils::StreamLogger& operator<<(core::utils::StreamLogger& logger,
                                      const uint8_t byte) {
  return logger << "0x" << std::setfill('0') << std::setw(2) << std::hex
                << static_cast<int>(byte);
}

}  // namespace

Mpu9250::Mpu9250(const Config& config,
                 std::unique_ptr<comm::CommunicationAbs> comm,
                 std::unique_ptr<core::utils::Node> node)
  : ImuSensorModule(ImuType, SensorName, true)
  , config_(config)
  , comm_{std::move(comm)}
  , node_{std::move(node)}
  , mag_spec_{CreateSensorSpecs(MagScaleMap.at(config.mag_scale.value()), 1.0F)}
  , gyro_spec_{CreateSensorSpecs(GyroScaleMap.at(config.gyro_scale.value()),
                                 DEG_TO_RAD)}
  , accel_spec_{CreateSensorSpecs(AccelScaleMap.at(config.accel_scale.value()),
                                  GRAVITY)}
  , temp_spec_{SensorSpecs<1>(TempSensitivity, 1.0F)} {
  temp_spec_.SetCalibration(CreateScalar(1), CreateScalar(kTempBias),
                            CreateScalar(kTempOffset));
  // ReadCalibrationFile();
}

bool Mpu9250::ProbeMpu() const {
  const uint8_t res = ReadRegister(mpu9250::WHO_AM_I);
  if (res != mpu9250::WHO_AM_I_RESPONSE) {
    node_->GetNodeLogger()->Warn("Bad IMU device ID, ")
      << "expect: " << mpu9250::WHO_AM_I_RESPONSE << " but received: " << res;
    return false;
  }
  return true;
}

bool Mpu9250::ProbeAk8963() const {
  ConfigureI2C();
  const uint8_t res = ReadAK8963Register(ak8963::WHO_AM_I);
  if (res != ak8963::WHO_AM_I_RESPONSE) {
    node_->GetNodeLogger()->Warn("Bad Magnetometer device ID, ")
      << "expect: " << ak8963::WHO_AM_I_RESPONSE << " but received: " << res;
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

bool Mpu9250::Reset() const {
  return ResetMpu() && RestAk8963();
}

bool Mpu9250::ResetMpu() const {
  // reset all registers to reset bit (7)
  if (not WriteRegister(mpu9250::PWR_MGMT_1, mpu9250::RESET)) {
    return false;
  }
  MilliDelay(LongDelay);

  // Auto select clock source to be PLL gyroscope, else internal 20MHz
  if (not WriteRegister(mpu9250::PWR_MGMT_1, mpu9250::AUTO_CLK_SRC)) {
    return false;
  }
  MilliDelay(ShortDelay);

  // Enable Acc & Gyro
  constexpr auto enable_byte = 0x00;
  if (not WriteRegister(mpu9250::PWR_MGMT_2, enable_byte)) {
    return false;
  }
  MilliDelay(ShortDelay);
  return true;
}

void Mpu9250::Initialize() {
  ResetMpu();

  // The sensor is set to 1 kHz sample rates, but all these rates are further
  // reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
  // Set sample rate = sensor output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  // disable fifo
  WriteRegister(mpu9250::FIFO_EN, mpu9250::DISABLE_FIFO);

  InitializeGyro();
  InitializeAccel();
  ConfigureI2C();
  InitializeMag();

  ValidateConfiguration();
  initialized_ = true;
}

bool Mpu9250::Test() {
  return false;
}

bool Mpu9250::InitializeAccel() const {
  return SetAccelScale() && SetAccelBW();
}

bool Mpu9250::SetAccelBW() const {
  // Set accelerometer sample rate configuration
  const auto bw = ConfigValueOr("Accel BW", config_.accel_bw, DefaultAccelBW);
  const auto bw_byte = ToByte(bw);
  constexpr auto mask2 = 0xF0;
  return SetRegisterByte(mpu9250::ACCEL_CONFIG2, bw_byte, mask2);
}

bool Mpu9250::SetAccelScale() const {
  // Set accelerometer full-scale range configuration
  const auto scale =
    ConfigValueOr("Accel Scale", config_.accel_scale, DefaultAccelScale);
  const auto scale_byte = ToByte(scale);
  constexpr auto mask1 = 0xE7;
  return SetRegisterByte(mpu9250::ACCEL_CONFIG, scale_byte, mask1);
}

bool Mpu9250::InitializeGyro() const {
  return SetGyroBW() && SetGyroScale();
}

bool Mpu9250::SetGyroBW() const {
  // Configure Gyro and Thermometer bandwidth
  const auto bw = ConfigValueOr("Gyro BW", config_.gyro_bw, DefaultGyroBW);
  const auto bw_byte = ToByte(bw);
  constexpr auto cmask = 0xF8;
  return SetRegisterByte(mpu9250::CONFIG, bw_byte, cmask);
}

bool Mpu9250::SetGyroScale() const {
  const auto scale =
    ConfigValueOr("Gyro Scale", config_.gyro_scale, DefaultGyroScale);
  const auto scale_byte = ToByte(scale);
  constexpr auto gmask = 0xE4;
  return SetRegisterByte(mpu9250::GYRO_CONFIG, scale_byte, gmask);
}

bool Mpu9250::InitializeMag() const {
  return RestAk8963() && SetMagMode(config_.mag_mode) && SetMagScale();
}

bool Mpu9250::RestAk8963() const {
  return WriteAK8963Register(ak8963::CNTL2, ak8963::RESET);
}

bool Mpu9250::SetMagScale() const {
  const auto scale =
    ConfigValueOr("Mag Scale", config_.mag_scale, DefaultMagScale);
  const auto scale_byte = ToByte(scale);
  constexpr auto cmask = 0xEF;
  return SetAK8963RegisterByte(ak8963::CNTL1, scale_byte, cmask);
  // MilliDelay(ShortDelay);
}

bool Mpu9250::SetMagMode(const std::optional<MagMode> mag_mode) const {
  const auto mode = ConfigValueOr("Mag Mode", mag_mode, DefaultMagMode);
  const auto mode_byte = ToByte(mode);
  constexpr auto cmask = 0xF0;
  return SetAK8963RegisterByte(ak8963::CNTL1, mode_byte, cmask);
  // MilliDelay(ShortDelay);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues() {
  SetMagMode(MagMode::POWER_DOWN);
  SetMagMode(MagMode::FUSE_ROM_ACCESS);

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

  // re-initialize magnetometer
  InitializeMag();
}

std::optional<AccelScale> Mpu9250::ReadAccelScale() const {
  const uint8_t data = ReadRegister(mpu9250::ACCEL_CONFIG);
  constexpr uint8_t mask = 0x18;  // mask bits [4:3]
  const uint8_t fs = static_cast<uint8_t>(data & mask);
  printf("AccelScale 0x%x\n", fs);
  return AccelScaleConfigMap.FindKey(fs);
}

std::optional<AccelBW> Mpu9250::ReadAccelBW() const {
  const uint8_t data = ReadRegister(mpu9250::ACCEL_CONFIG2);
  constexpr uint8_t mask3_f_inv = 0x08;  // mask bits [3]
  const uint8_t f_inv = static_cast<uint8_t>(data & mask3_f_inv);
  constexpr uint8_t mask_bw = 0x07;  // mask bits [2:0]
  const uint8_t bw = static_cast<uint8_t>(data & mask_bw);
  printf("AccelBW 0x%x\n", bw);
  if (f_inv != 0) {
    node_->GetNodeLogger()->Error("Accelerometer fChoice is disabled");
  }
  return AccelBWConfigMap.FindKey(bw);
}

std::optional<GyroBW> Mpu9250::ReadGyroBW() const {
  const uint8_t data = ReadRegister(mpu9250::CONFIG);
  constexpr uint8_t mask_bw = 0x07;  // mask bits [2:0]
  const uint8_t bw = static_cast<uint8_t>(data & mask_bw);
  printf("GyroBW 0x%x\n", bw);
  return GyroBWConfigMap.FindKey(bw);
}

std::optional<GyroScale> Mpu9250::ReadGyroScale() const {
  const uint8_t data = ReadRegister(mpu9250::GYRO_CONFIG);
  constexpr uint8_t mask3_f_inv = 0x03;  // mask bits [1:0]
  const uint8_t f_inv = static_cast<uint8_t>(data & mask3_f_inv);
  constexpr uint8_t mask_fs = 0x18;  // mask bits [4:3]
  const uint8_t fs = static_cast<uint8_t>(data & mask_fs);
  if (f_inv != 0) {
    node_->GetNodeLogger()->Error("Gyroscope fChoice is disabled");
  }
  printf("GyroScale 0x%x\n", fs);
  return GyroScaleConfigMap.FindKey(fs);
}

std::optional<MagMode> Mpu9250::ReadMagMode() const {
  const uint8_t data = ReadAK8963Register(ak8963::CNTL1);
  constexpr uint8_t mask = 0x0F;  // mask bits[3:0]
  const uint8_t mode = static_cast<uint8_t>(data & mask);
  printf("MagMode 0x%x\n", mode);
  return MagModeConfigMap.FindKey(mode);
}

std::optional<MagScale> Mpu9250::ReadMagScale() const {
  const uint8_t data = ReadAK8963Register(ak8963::CNTL1);
  constexpr uint8_t mask = 0x10;  // mask res bit[4]
  const uint8_t scale = static_cast<uint8_t>(data & mask);
  printf("MagScale 0x%x\n", scale);
  return MagScaleConfigMap.FindKey(scale);
}

std::pair<bool, Config> Mpu9250::ValidateConfiguration() const {
  Config actual_cfg;

  bool valid = true;

  actual_cfg.accel_scale = ReadAccelScale();
  actual_cfg.accel_bw = ReadAccelBW();
  actual_cfg.gyro_bw = ReadGyroBW();
  actual_cfg.gyro_scale = ReadGyroScale();
  actual_cfg.mag_mode = ReadMagMode();
  actual_cfg.mag_scale = ReadMagScale();
  actual_cfg.sample_rate_divisor = ReadRegister(mpu9250::SMPLRT_DIV);

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

bool Mpu9250::Calibrate() {
  return CalibrateAccelerometer() && CalibrateGyroscope() &&
         CalibrateMagnetometer();
}

bool Mpu9250::CalibrateAccelerometer() {
  if (not initialized_) {
    SYS_LOG_WARN("Failed to calibrate Accelerometer: Sensor not initalized");
    return false;
  }

  auto read_accel_data = [this]() { return ReadRawData().accel; };
  const auto result = hardware::common::sensors::CalibrateAccelerometer(
    read_accel_data, accel_spec_);
  if (not result.has_value()) {
    SYS_LOG_WARN("Failed to calibrate Accelerometer");
    return false;
  }
  accel_spec_ = result.value();
  return true;
}

bool Mpu9250::CalibrateGyroscope() {
  if (not initialized_) {
    SYS_LOG_WARN("Failed to calibrate gyroscope: Sensor not initalized");
    return false;
  }

  auto read_gyro_data = [this]() { return ReadRawData().gyro; };

  const auto result =
    hardware::common::sensors::CalibrateGyroscope(read_gyro_data, gyro_spec_);
  if (not result.has_value()) {
    SYS_LOG_WARN("Failed to calibrate gyroscope");
    return false;
  }
  gyro_spec_ = result.value();
  return true;
}

bool Mpu9250::CalibrateMagnetometer() {
  if (not initialized_) {
    SYS_LOG_WARN("Failed to calibrate magnetometer: Sensor not initalized");
    return false;
  }

  auto read_mag_data = [this]() { return ReadRawData().mag; };
  const auto result =
    hardware::common::sensors::CalibrateMagnetometer(read_mag_data, mag_spec_);
  if (not result.has_value()) {
    SYS_LOG_WARN("Failed to calibrate magnetometer");
    return false;
  }
  mag_spec_ = result.value();
  return true;
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

  accel_spec_.SetCalibration(accel_misalignment, accel_bias, Vec3::Zero());
  gyro_spec_.SetCalibration(Mat3::Identity(), gyro_bias, Vec3::Zero());
}

void Mpu9250::Update() {
  raw_ = ReadRawData();
  const ImuData imu = ApplySensorSpecs(raw_);
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

SensorRawData Mpu9250::GetRawData() const {
  return raw_;
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
  raw.accel = Vec3From16BitsVector(full_bits.begin() + accel_start_idx);
  constexpr auto gyro_start_idx = 4;
  raw.gyro = Vec3From16BitsVector(full_bits.begin() + gyro_start_idx);
  constexpr auto mag_start_idx = 7;
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
  imu.overflow = raw.mag_over_flow;
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

bool Mpu9250::RequestReadAK8963Registers(uint8_t reg, uint8_t count) const {
  // set slave 0 to the AK8963 and set for read
  constexpr uint8_t read_address = ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG;
  if (not WriteRegister(mpu9250::I2C_SLV0_ADDR, read_address)) {
    return false;
  }
  // set the register to the desired AK8963 sub address
  if (not WriteRegister(mpu9250::I2C_SLV0_REG, reg)) {
    return false;
  }
  // enable I2C and request the bytes
  if (not WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count)) {
    return false;
  }
  MilliDelay(5);
  return true;
}

std::vector<uint8_t> Mpu9250::ReadAK8963Registers(uint8_t reg,
                                                  uint8_t count) const {
  RequestReadAK8963Registers(reg, count);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  return ReadRegisters(mpu9250::EXT_SENS_DATA_00, count);
}

bool Mpu9250::SetRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const {
  const uint8_t current = ReadRegister(reg);
  const bool res = WriteRegister(reg, SetFlags(current, mask, byte));
  printf("was 0x%x -> 0x%x\n", current, ReadRegister(reg));
  return res;
}

bool Mpu9250::SetAK8963RegisterByte(uint8_t reg, uint8_t byte,
                                    uint8_t mask) const {
  const uint8_t current = ReadAK8963Register(reg);
  const bool res = WriteAK8963Register(reg, SetFlags(current, mask, byte));
  printf("was 0x%x -> 0x%x\n", current, ReadAK8963Register(reg));
  return res;
}

bool Mpu9250::WriteRegister(uint8_t reg, uint8_t data) const {
  return comm_->WriteByte(reg, data);
  MicroDelay(10);
}

bool Mpu9250::WriteAK8963Register(uint8_t reg, uint8_t data) const {
  // set slave 0 to the AK8963 and set for write
  if (not WriteRegister(mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR)) {
    return false;
  }
  // set the register to the desired AK8963 sub address
  if (not WriteRegister(mpu9250::I2C_SLV0_REG, reg)) {
    return false;
  }
  // store the data for write
  if (not WriteRegister(mpu9250::I2C_SLV0_DO, data)) {
    return false;
  }
  // enable I2C and send 1 byte
  constexpr uint8_t count = 1;
  const bool res =
    WriteRegister(mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count);
  MilliDelay(5);
  printf("write reg: 0x%x data: 0x%x\n", reg, data);
  return res;
}
}  // namespace hardware::common::sensors::mpu
   // hardware::common::sensors::mpu
