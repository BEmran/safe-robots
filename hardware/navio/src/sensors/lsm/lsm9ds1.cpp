// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "sensors/lsm/lsm9ds1.hpp"

#include "navio/hardware_utils.hpp"
#include "navio/spi.hpp"
#include "sensors/common/calibrate.hpp"

namespace sensors::lsm {
namespace cu = common::utils;
using namespace cu::literals;  // NOLINT [build/namespaces_literals] TODO(Bara)
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
cu::SensorSpecs<3> CreateSensorSpecs(const cu::MATH_TYPE scale,
                                     const cu::MATH_TYPE unit) {
  const auto sen = kMaxBitVal / scale;
  return cu::SensorSpecs<3>(sen, unit);
}

std::string ConfigToString(const Config& cfg) {
  std::stringstream ss;
  ss << "\tAccelerometer:"                                             //
     << " Scale: " << AccelScaleMap()[cfg.accel_scale].name            //
     << ", Sampling rate: " << AccelODRMap()[cfg.accel_odr]            //
     << ", Anti alias: " << AccelAntiAliasMap()[cfg.accel_anti_alias]  //
     << ", High resolution bw: "
     << AccelHighResBWMap()[cfg.accel_high_res_bw]          //
     << "\n\tGyroscope:"                                    //
     << " Sampling rate: " << GyroODRMap()[cfg.gyro_odr]    //
     << ", Scale: " << GyroScaleMap()[cfg.gyro_scale].name  //
     << "\n\tMagnetometer:"                                 //
     << " Sampling rate: " << MagODRMap()[cfg.mag_odr]      //
     << ", XY mode: " << MagXYModeMap()[cfg.mag_xy_mode]    //
     << ", Z mode: " << MagZModeMap()[cfg.mag_z_mode]       //
     << ", Operating mode: " << MagOperatingModeMap()[cfg.mag_operating_mode]
     << ", Scale: " << MagScaleMap()[cfg.mag_scale].name;  //
  return ss.str();
}

}  // namespace

Lsm9ds1::Lsm9ds1(const Config& config, std::unique_ptr<navio::SPI> comm_ag,
                 std::unique_ptr<navio::SPI> comm_mag,
                 std::unique_ptr<core::utils::Node> node)
  : cu::ImuSensorModule(ImuType, SensorName, true)
  , config_(config)
  , comm_ag_{std::move(comm_ag)}
  , comm_mag_{std::move(comm_mag)}
  , node_{std::move(node)}
  , mag_spec_{CreateSensorSpecs(MagScaleMap()[config.mag_scale].value,
                                kMagUnit)}
  , gyro_spec_{CreateSensorSpecs(GyroScaleMap()[config.gyro_scale].value,
                                 cu::DEG_TO_RAD)}
  , accel_spec_{CreateSensorSpecs(AccelScaleMap()[config.accel_scale].value,
                                  cu::GRAVITY)}
  , temp_spec_{cu::SensorSpecs<1>(kTempSensitivity, 1.F)} {
  temp_spec_.SetCalibration(cu::CreateScalar(1), cu::CreateScalar(kTempBias),
                            cu::CreateScalar(kTempOffset));
  // ReadCalibrationFile();
}

void Lsm9ds1::ReadCalibrationFile() {
  // TODO(Bara) read from config file instead
  cu::Mat3 accel_misalignment;

  accel_misalignment << (cu::Mat3() << 0.998122F, 0.00794836F, 0.000548448F,
                         -0.00552448F, 0.998181F, -0.00669443F, 0.0189156F,
                         0.00407755F, 0.993244F)
                          .finished();
  cu::Vec3 accel_bias(-0.00387028F, -0.0128085F, 0.0108167F);
  cu::Vec3 gyro_bias(12.629F, 7.572F, -9.618F);

  accel_spec_.SetCalibration(accel_misalignment, accel_bias,
                             cu::Vec3::Zero());  //
  gyro_spec_.SetCalibration(cu::Mat3::Identity(), gyro_bias,
                            cu::Vec3::Zero());  //
}

bool Lsm9ds1::ProbeAG() const {
  auto a = ReadAGRegister(xg::WHO_AM_I);
  std::cout << a << "\t" << int(a) << std::endl;
  if (ReadAGRegister(xg::WHO_AM_I) != xg::WHO_AM_I_RESPONSE) {
    node_->GetLogger().Warn("Bad IMU device ID");
    return false;
  }
  return true;
}

bool Lsm9ds1::ProbeMag() const {
  auto a = ReadMagRegister(mag::WHO_AM_I);
  std::cout << a << "\t" << int(a) << std::endl;
  if (ReadMagRegister(mag::WHO_AM_I) != mag::WHO_AM_I_RESPONSE) {
    node_->GetLogger().Warn("Bad Magnetometer device ID");
    return false;
  }
  return true;
}

bool Lsm9ds1::Probe() {
  if (ProbeAG() && ProbeMag()) {
    node_->GetLogger().Debug("LSM9DS1 is online!");
    return true;
  }
  return false;
}

void Lsm9ds1::Reset() const {
  // reset all registers
  constexpr auto boot = 0x80;   // bit[7]
  constexpr auto reset = 0x01;  // bit[0]
  const auto byte = static_cast<uint8_t>(reset | boot);
  WriteAGRegister(xg::CTRL_REG8, byte);
  navio::hardware_utils::Delay(LongDelay);

  // disable fifo
  constexpr auto fifo_disable = 0x00_uc;
  WriteAGRegister(xg::CTRL_REG9, fifo_disable);
  navio::hardware_utils::Delay(ShortDelay);
}

void Lsm9ds1::Initialize() {
  // Reset();

  InitializeGyro();
  InitializeAccel();
  InitializeMag();

  ValidateConfiguration();
}

bool Lsm9ds1::Test() {
  return false;
}

void Lsm9ds1::InitializeAccel() const {
  // Configure Output data rate, scale and bw
  const auto odr = cu::ToByte(config_.accel_odr);      // bits[7:5]
  const auto scale = cu::ToByte(config_.accel_scale);  // bits[4:3]
  const auto bw_select = config_.accel_bw_selection ?
                           0x04_uc :
                           0x00_uc;  // determine bw selection bit[2]
  const auto bw_aa = config_.accel_bw_selection ?
                       cu::ToByte(config_.accel_anti_alias) :
                       0x00_uc;  // bit[1:0]
  const auto byte1 = static_cast<uint8_t>(odr | scale | bw_select | bw_aa);
  WriteAGRegister(xg::CTRL_REG6_XL, byte1);

  // configure high resolution band width
  const auto hr_enabled =
    config_.accel_high_res_enabled ? 0x80_uc : 0x00_uc;  // bit[7]
  const auto hr_bw = config_.accel_high_res_enabled ?
                       cu::ToByte(config_.accel_high_res_bw) :
                       0x00_uc;  // bits[6:5]
  const auto byte2 = static_cast<uint8_t>(hr_enabled | hr_bw);
  constexpr auto mask_hr_bw = 0xE0_uc;
  SetAGRegisterByte(xg::CTRL_REG7_XL, byte2, mask_hr_bw);

  // enable all axis
  const auto enable = 0x38_uc;  // enable all axis bits[5:3]
  constexpr auto mask_enable = 0xC7_uc;
  SetAGRegisterByte(xg::CTRL_REG5_XL, enable, mask_enable);
}

void Lsm9ds1::InitializeGyro() const {
  // Configure Output data rate and scale
  const auto odr = cu::ToByte(config_.gyro_odr);      // bits[7:5]
  const auto scale = cu::ToByte(config_.gyro_scale);  // bits[4:3]
  const auto bw = 0x00_uc;                            // default bits[1:0]
  const auto byte1 = static_cast<uint8_t>(odr | scale | bw);
  WriteAGRegister(xg::CTRL_REG1_G, byte1);

  // Configure interrupt
  const auto byte2 = 0x00_uc;  // disabled
  WriteAGRegister(xg::CTRL_REG2_G, byte2);

  // Configure mode and HPF cutoff frequency
  const auto lp_mode = 0x00_uc;      // disabled bit[7]
  const auto hp_enable = 0x00_uc;    // disabled bit[6]
  const auto hpf_cut_off = 0x00_uc;  // disabled bits[3:0]
  const auto byte3 = static_cast<uint8_t>(lp_mode | hp_enable | hpf_cut_off);
  WriteAGRegister(xg::CTRL_REG3_G, byte3);

  // enable all axis
  const auto enable = 0x38_uc;  // enable all axis bits[5:3]
  const auto latch = 0x02_uc;   // enable latch Interrupt bit[1]
  const auto byte4 = static_cast<uint8_t>(enable | latch);
  WriteAGRegister(xg::CTRL_REG4, byte4);

  // Configure directional user orientation selection
  const auto ori = 0x00_uc;  // all positive no flip
  WriteAGRegister(xg::ORIENT_CFG_G, ori);
}

void Lsm9ds1::InitializeMag() const {
  // configure xy axis mode and temperature compensation
  const auto temp_comp =
    static_cast<uint8_t>(config_.mag_temp_compensation);  // bit[7]
  const auto xy_mode = cu::ToByte(config_.mag_xy_mode);   // bits[6:5]
  const auto odr = cu::ToByte(config_.mag_odr);           // bits[4:2]
  const auto byte1 = static_cast<uint8_t>(temp_comp | xy_mode | odr);
  WriteMagRegister(mag::CTRL_REG1_M, byte1);

  // configure scale
  const auto scale = cu::ToByte(config_.mag_scale);  // bits[6:5]
  WriteMagRegister(mag::CTRL_REG2_M, scale);

  // configure operating mode
  const auto lower_power = 0x00_uc;  // bit[5]
  const auto spi = 0x00_uc;  // 0x04_uc;          // read/write enable bit[2]
  const auto o_mode = cu::ToByte(config_.mag_operating_mode);  // bits[1:0]
  const auto byte2 = static_cast<uint8_t>(lower_power | spi | o_mode);
  WriteMagRegister(mag::CTRL_REG3_M, byte2);

  // configure z axis mode
  const auto z_mode = cu::ToByte(config_.mag_z_mode);  // bits[3:2]
  const auto little_endian = 0x00_uc;                  // bit[1]
  const auto byte3 = static_cast<uint8_t>(z_mode | little_endian);
  WriteMagRegister(mag::CTRL_REG4_M, byte3);

  // configure update to continuous (no data blocking)
  const auto continuos = 0x00_uc;
  WriteMagRegister(mag::CTRL_REG5_M, continuos);
}

AccelScale Lsm9ds1::ReadAccelScale() const {
  const auto data = ReadAGRegister(xg::CTRL_REG6_XL);
  constexpr auto mask = 0x18_uc;  // mask bits [4:3]
  const auto scale = static_cast<uint8_t>(data & mask);
  return cu::Find(AccelScaleMap(), scale);
}

AccelODR Lsm9ds1::ReadAccelODR() const {
  const auto data = ReadAGRegister(xg::CTRL_REG6_XL);
  constexpr auto mask = 0xE0_uc;  // mask bits [7:5]
  const auto odr = static_cast<uint8_t>(data & mask);
  return cu::Find(AccelODRMap(), odr);
}

std::pair<bool, AccelAntiAliasingBW> Lsm9ds1::ReadAccelAccelAntiAlias() const {
  const auto data = ReadAGRegister(xg::CTRL_REG6_XL);
  constexpr auto mask = 0x03_uc;  // mask bits [1:0]
  const auto aa = static_cast<uint8_t>(data & mask);
  constexpr auto mask_select = 0x04_uc;  // mask bit [2]
  const auto bw_select = static_cast<bool>(data & mask_select);
  return {bw_select, cu::Find(AccelAntiAliasMap(), aa)};
}

std::pair<bool, AccelHighResolutionBW> Lsm9ds1::ReadAccelHighResBW() const {
  const auto data = ReadAGRegister(xg::CTRL_REG7_XL);
  constexpr auto mask = 0x60_uc;  // mask bits [6:5]
  const auto bw = static_cast<uint8_t>(data & mask);
  constexpr auto hr_mask = 0x80_uc;
  const auto hr_enabled = static_cast<bool>(data & hr_mask);

  return {hr_enabled, cu::Find(AccelHighResBWMap(), bw)};
}

GyroODR Lsm9ds1::ReadGyroODR() const {
  const auto data = ReadAGRegister(xg::CTRL_REG1_G);
  constexpr auto mask = 0xE0_uc;  // mask bits [7:5]
  const auto odr = static_cast<uint8_t>(data & mask);
  return cu::Find(GyroODRMap(), odr);
}

GyroScale Lsm9ds1::ReadGyroScale() const {
  const auto data = ReadAGRegister(xg::CTRL_REG1_G);
  constexpr auto mask = 0x18_uc;  // mask bits [4:3]
  const auto scale = static_cast<uint8_t>(data & mask);
  return Find(GyroScaleMap(), scale);
}

MagODR Lsm9ds1::ReadMagODR() const {
  const auto data = ReadMagRegister(mag::CTRL_REG1_M);
  constexpr auto mask = 0x1C_uc;  // mask bits[4:2]
  const auto odr = static_cast<uint8_t>(data & mask);
  return cu::Find(MagODRMap(), odr);
}

MagXYMode Lsm9ds1::ReadMagXYMode() const {
  const auto data = ReadMagRegister(mag::CTRL_REG1_M);
  constexpr auto mask = 0x60_uc;  // mask bits[6:5]
  const auto mode = static_cast<uint8_t>(data & mask);
  return cu::Find(MagXYModeMap(), mode);
}

MagZMode Lsm9ds1::ReadMagZMode() const {
  const auto data = ReadMagRegister(mag::CTRL_REG4_M);
  constexpr auto mask = 0x0C_uc;  // mask bits[3:2]
  const auto mode = static_cast<uint8_t>(data & mask);
  return cu::Find(MagZModeMap(), mode);
}

MagOperatingMode Lsm9ds1::ReadMagOperationMode() const {
  const auto data = ReadMagRegister(mag::CTRL_REG3_M);
  constexpr auto mask = 0x03_uc;  // mask bits[1:0]
  const auto mode = static_cast<uint8_t>(data & mask);
  return cu::Find(MagOperatingModeMap(), mode);
}

MagScale Lsm9ds1::ReadMagScale() const {
  const auto data = ReadMagRegister(mag::CTRL_REG2_M);
  constexpr auto mask = 0x60_uc;  // mask bit[6:5]
  const auto scale = static_cast<uint8_t>(data & mask);
  return Find(MagScaleMap(), scale);
}

std::pair<bool, Config> Lsm9ds1::ValidateConfiguration() const {
  Config actual_cfg;
  actual_cfg.accel_scale = ReadAccelScale();
  actual_cfg.accel_odr = ReadAccelODR();
  std::tie(actual_cfg.accel_bw_selection, actual_cfg.accel_anti_alias) =
    ReadAccelAccelAntiAlias();
  std::tie(actual_cfg.accel_high_res_enabled, actual_cfg.accel_high_res_bw) =
    ReadAccelHighResBW();
  actual_cfg.gyro_odr = ReadGyroODR();
  actual_cfg.gyro_scale = ReadGyroScale();
  actual_cfg.mag_odr = ReadMagODR();
  actual_cfg.mag_xy_mode = ReadMagXYMode();
  actual_cfg.mag_z_mode = ReadMagZMode();
  actual_cfg.mag_operating_mode = ReadMagOperationMode();
  actual_cfg.mag_scale = ReadMagScale();

  bool valid = true;
  valid &= actual_cfg.accel_scale == config_.accel_scale;
  valid &= actual_cfg.accel_odr == config_.accel_odr;
  valid &= actual_cfg.gyro_odr == config_.gyro_odr;
  valid &= actual_cfg.gyro_scale == config_.gyro_scale;
  valid &= actual_cfg.mag_odr == config_.mag_odr;
  valid &= actual_cfg.mag_xy_mode == config_.mag_xy_mode;
  valid &= actual_cfg.mag_z_mode == config_.mag_z_mode;
  valid &= actual_cfg.mag_operating_mode == config_.mag_operating_mode;
  valid &= actual_cfg.mag_scale == config_.mag_scale;

  valid &= actual_cfg.accel_bw_selection == config_.accel_bw_selection;
  if (valid && config_.accel_bw_selection) {
    valid &= actual_cfg.accel_anti_alias == config_.accel_anti_alias;
  }

  valid &= actual_cfg.accel_high_res_enabled == config_.accel_high_res_enabled;
  if (valid && config_.accel_high_res_enabled) {
    valid &= actual_cfg.accel_high_res_bw == config_.accel_high_res_bw;
  }

  const auto config_str = ConfigToString(actual_cfg);
  const std::string valid_str = valid ? "successfully" : "failed to";
  const std::string msg = "LSM9DS1 " + valid_str +
                          " initialized! Actual configuration is set to:\n" +
                          config_str;
  valid ? node_->GetLogger().Debug(msg) : node_->GetLogger().Warn(msg);

  return {valid, actual_cfg};
}

void Lsm9ds1::Calibrate() {
  auto read_gyro_data = [this]() { return ReadRawData().gyro; };
  auto read_accel_data = [this]() { return ReadRawData().accel; };
  auto read_mag_data = [this]() { return ReadRawData().mag; };

  gyro_spec_ =
    common::calibrate::CalibrateGyroscope(read_gyro_data, gyro_spec_);
  accel_spec_ =
    common::calibrate::CalibrateAccelerometer(read_accel_data, accel_spec_);
  mag_spec_ =
    common::calibrate::CalibrateMagnetometer(read_mag_data, mag_spec_);
}

void Lsm9ds1::Update() {
  const auto raw = ReadRawData();
  cu::ImuData imu = ApplySensorSpecs(raw);
  imu.tait_bryan.data = cu::EstimateRPY(imu.accel.data);
  SetData(imu);
}

SensorRawData Lsm9ds1::ReadRawData() const {
  SensorRawData raw;
  raw.accel = ReadRawAccelData();
  raw.gyro = ReadRawGyroData();
  raw.mag = ReadRawMagData();
  raw.temp = ReadRawTemperatureData();
  return AlignAxis(raw);
}

cu::Vec3 Lsm9ds1::ReadRawAccelData() const {
  constexpr auto data_size = 6;
  const auto data = ReadAGRegisters(xg::OUT_X_L_XL, data_size);
  const auto full_bits = ExtractFullBits(data);
  return cu::Vec3From16BitsVector(full_bits.begin());
}

cu::Vec3 Lsm9ds1::ReadRawGyroData() const {
  constexpr auto data_size = 6;
  const auto data = ReadAGRegisters(xg::OUT_X_L_G, data_size);
  const auto full_bits = ExtractFullBits(data);
  return cu::Vec3From16BitsVector(full_bits.begin());
}

cu::Vec3 Lsm9ds1::ReadRawMagData() const {
  constexpr auto data_size = 6;
  const auto data = ReadMagRegisters(mag::OUT_X_L_M, data_size);
  const auto full_bits = ExtractFullBits(data);
  return cu::Vec3From16BitsVector(full_bits.begin());
}

cu::MATH_TYPE Lsm9ds1::ReadRawTemperatureData() const {
  constexpr auto data_size = 2;
  const auto data = ReadAGRegisters(xg::OUT_TEMP_L, data_size);
  const auto full_bits = ExtractFullBits(data).front();
  return static_cast<cu::MATH_TYPE>(full_bits);
}

std::vector<int16_t>
Lsm9ds1::ExtractFullBits(const std::vector<uint8_t>& data) {
  std::vector<int16_t> full_bits(data.size() / 2);
  for (size_t i = 0; i < full_bits.size(); i++) {
    full_bits[i] = cu::To16Bit(data[i * 2 + 1], data[i * 2]);
  }
  return full_bits;
}

SensorRawData Lsm9ds1::AlignAxis(const SensorRawData& raw) {
  SensorRawData aligned = raw;
  std::swap(aligned.accel.x(), aligned.accel.y());
  std::swap(aligned.gyro.x(), aligned.gyro.y());
  aligned.accel.x() *= -1;
  aligned.accel.y() *= -1;
  aligned.gyro.x() *= -1;
  aligned.gyro.y() *= -1;
  aligned.mag.y() *= -1;
  aligned.mag.z() *= -1;
  return aligned;
}

cu::ImuData Lsm9ds1::ApplySensorSpecs(const SensorRawData& raw) const {
  cu::ImuData imu;
  imu.accel.data = accel_spec_.Apply(raw.accel);
  imu.gyro.data = gyro_spec_.Apply(raw.gyro);
  imu.mag.data = mag_spec_.Apply(raw.mag);
  imu.temp.value = temp_spec_.Apply(raw.temp);
  return imu;
}

uint8_t Lsm9ds1::ReadAGRegister(uint8_t reg) const {
  return ReadAGRegisters(reg, 1)[0];
}

std::vector<uint8_t> Lsm9ds1::ReadAGRegisters(uint8_t reg,
                                              uint8_t count) const {
  return comm_ag_->ReadRegisters(reg, count);
}

uint8_t Lsm9ds1::ReadMagRegister(uint8_t reg) const {
  return ReadMagRegisters(reg, 1)[0];
}

std::vector<uint8_t> Lsm9ds1::ReadMagRegisters(uint8_t reg,
                                               uint8_t count) const {
  if (count > 1) {
    reg = static_cast<uint8_t>(reg | 0x40_uc);
  }
  return comm_mag_->ReadRegisters(reg | 0x40_uc, count);
}

void Lsm9ds1::WriteAGRegister(uint8_t reg, uint8_t data) const {
  comm_ag_->WriteRegister(reg, data);
}

void Lsm9ds1::WriteMagRegister(uint8_t reg, uint8_t data) const {
  comm_mag_->WriteRegister(reg, data);
}

void Lsm9ds1::SetAGRegisterByte(uint8_t reg, uint8_t byte, uint8_t mask) const {
  const auto current = ReadAGRegister(reg);
  WriteAGRegister(reg, cu::SetFlags(current, mask, byte));
}

void Lsm9ds1::SetMagRegisterByte(uint8_t reg, uint8_t byte,
                                 uint8_t mask) const {
  const auto current = ReadMagRegister(reg);
  WriteMagRegister(reg, cu::SetFlags(current, mask, byte));
}

}  // namespace sensors::lsm
