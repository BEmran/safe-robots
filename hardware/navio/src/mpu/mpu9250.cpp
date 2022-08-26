#include "mpu/mpu9250.hpp"
#include "mpu/spi.hpp"
#include "mpu/calibrate.hpp"

namespace mpu
{
namespace
{

using SensorModuleType = core::sensors::SensorModuleType;
constexpr auto ImuType = SensorModuleType::IMU;
constexpr auto GyroType = SensorModuleType::GYROSCOPE;
constexpr auto MagType = SensorModuleType::MAGNETOMETER;
constexpr auto AccelType = SensorModuleType::ACCELEROMETER;
constexpr auto TempType = SensorModuleType::TEMPERATURE;
constexpr auto TempScale = 333.87F;
const auto kTempBias = Vec3::Ones() * 21.F;
const auto kTempOffset = Vec3::Ones() * 21.F;

template <typename T,
          typename =
              typename std::enable_if<std::is_arithmetic<T>::value, T>::type>
struct SpecInfo
{
  T value;
  uint8_t byte;
  std::string name;
  SpecInfo() : SpecInfo(0, 0_uc, ""){};
  SpecInfo(const T value_, const uint8_t byte_, const std::string& name_)
    : value(value_), byte(byte_), name(name_){};
};

template <typename E, typename T>
class SpecInfoMap
{
  typedef std::map<E, SpecInfo<T>> Map;
  using Iterator = typename Map::iterator;

 public:
  explicit SpecInfoMap(const Map& map) : map_(map)
  {
  }

  E Find(const uint8_t byte)
  {
    auto ptr = std::find_if(map_.begin(), map_.end(), [byte](auto ele) {
      return byte == ele.second.byte;
    });
    if (ptr == map_.end())
    {
      throw std::runtime_error("undefined specification");
    }
    return ptr->first;
  }

  SpecInfo<T> operator[](const E key)
  {
    return map_[key];
  }

 private:
  Map map_;
};

SpecInfoMap<AccelBandWidthHz, uint8_t> accel_bw_map({
    {AccelBandWidthHz::BW_218HZ, {0, 0x01_uc, "218 HZ"}},  //
    {AccelBandWidthHz::BW_99HZ, {0, 0x02_uc, "99 HZ"}},    //
    {AccelBandWidthHz::BW_44HZ, {0, 0x03_uc, "44 HZ"}},    //
    {AccelBandWidthHz::BW_21HZ, {0, 0x04_uc, "21 HZ"}},    //
    {AccelBandWidthHz::BW_10HZ, {0, 0x05_uc, "10 HZ"}},    //
    {AccelBandWidthHz::BW_5HZ, {0, 0x06_uc, "5 HZ"}}       //
});

SpecInfoMap<AccelScale, float> accel_scale_map({
    {AccelScale::FS_2G, {2.F, 0x00_uc, "2G"}},    //
    {AccelScale::FS_4G, {4.F, 0x08_uc, "4G"}},    //
    {AccelScale::FS_8G, {8.F, 0x10_uc, "8G"}},    //
    {AccelScale::FS_16G, {16.F, 0x18_uc, "16G"}}  //
});

SpecInfoMap<GyroBandWidthHz, uint8_t> gyro_bw_map({
    {GyroBandWidthHz::BW_250HZ, {0, 0x00_uc, "250 HZ"}},   //
    {GyroBandWidthHz::BW_184HZ, {0, 0x01_uc, "184 HZ"}},   //
    {GyroBandWidthHz::BW_92HZ, {0, 0x02_uc, "92 HZ"}},     //
    {GyroBandWidthHz::BW_41HZ, {0, 0x03_uc, "41 HZ"}},     //
    {GyroBandWidthHz::BW_20HZ, {0, 0x04_uc, "20 HZ"}},     //
    {GyroBandWidthHz::BW_10HZ, {0, 0x05_uc, "10 HZ"}},     //
    {GyroBandWidthHz::BW_5HZ, {0, 0x06_uc, "5 HZ"}},       //
    {GyroBandWidthHz::BW_3600HZ, {0, 0x07_uc, "3600 HZ"}}  //
});

SpecInfoMap<GyroScale, float> gyro_scale_map({
    {GyroScale::FS_250DPS, {250.F, 0x00_uc, "250 DPS"}},     //
    {GyroScale::FS_500DPS, {500.F, 0x08_uc, "500 DPS"}},     //
    {GyroScale::FS_1000DPS, {1000.F, 0x10_uc, "1000 DPS"}},  //
    {GyroScale::FS_2000DPS, {2000.F, 0x18_uc, "2000 DPS"}}   //
});

SpecInfoMap<MagScale, float> mag_scale_map({
    {MagScale::FS_14BITS, {0.25F * max_utesla, 0x00_uc, "14 BITS"}},  //
    {MagScale::FS_16BITS, {1.00F * max_utesla, 0x10_uc, "16 BITS"}}   //
});

SpecInfoMap<MagMode, uint8_t> mag_mode_map({
    {MagMode::POWER_DOWN, {0, 0x00_uc, "POWER DOWN"}},                  //
    {MagMode::SINGLE_MEASUREMENT, {0, 0x01_uc, "SINGLE MEASUREMENT"}},  //
    {MagMode::CONTINUES_8HZ, {0, 0x02_uc, "CONTINUES 8HZ"}},            //
    {MagMode::EXTERNAL_TRIGGER, {0, 0x04_uc, "EXTERNAL TRIGGER"}},      //
    {MagMode::CONTINUES_100HZ, {0, 0x06_uc, "CONTINUES 100HZ"}},        //
    {MagMode::SELF_TEST, {0, 0x08_uc, "SELF TEST"}},                    //
    {MagMode::FUSE_ROM_ACCESS, {0, 0x0F_uc, "FUSE ROM ACCESS"}}         //
});

uint8_t SetFlags(const uint8_t byte, const uint8_t mask, const uint8_t flag)
{
  const auto updated = (byte & mask) | flag;
  return static_cast<uint8_t>(updated);
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

Mpu9250::Mpu9250(const Config& config, std::unique_ptr<SPI> comm,
                 std::unique_ptr<core::utils::Node> node)
  : ImuSensorModule(ImuType, SensorName, true)
  , config_(config)
  , comm_{std::move(comm)}
  , node_{std::move(node)}

{
  sensor_specs_map[AccelType] =
      CreateSensorSpecs(accel_scale_map[config.accel_scale].value, GRAVITY);
  sensor_specs_map[GyroType] =
      CreateSensorSpecs(gyro_scale_map[config.gyro_scale].value, PI / 180.0F);
  sensor_specs_map[MagType] =
      CreateSensorSpecs(mag_scale_map[config.mag_scale].value, 1.0F);
  sensor_specs_map[TempType] =
      SensorSpecs(TempScale, 1.0F, kTempBias, kTempOffset);

  Mat3 accel_misalignment;
  accel_misalignment << 0.998122F, 0.00794836F, 0.000548448F,  //
      -0.00552448F, 0.998181F, -0.00669443F,                   //
      0.0189156F, 0.00407755F, 0.993244F;
  Vec3 accel_bias{-0.00387028F, -0.0128085F, 0.0108167F};
  Vec3 gyro_bias{12.629F, 7.572F, -9.618F};
  sensor_specs_map[AccelType].SetCalibration(accel_misalignment, accel_bias,
                                             Vec3::Zero());  //
  sensor_specs_map[GyroType].SetCalibration(Mat3::Identity(), gyro_bias,
                                            Vec3::Zero());  //
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
  // wake up device, clear sleep mode bit (6)
  // WriteRegister(mpu9250::PWR_MGMT_1, 0x00);
  // Delay(100);

  // reset all registers to reset bit (7)
  WriteRegister(mpu9250::PWR_MGMT_1, 0x80);
  Delay(100);

  // Auto select clock source to be PLL gyroscope reference, else internal
  // 20MHz
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
  const auto res = static_cast<uint8_t>(scale | mode);
  WriteAK8963Register(ak8963::CNTL, res);
  Delay(10);
}

void Mpu9250::ExtractMagnetometerSensitivityAdjustmentValues()
{
  WriteAK8963Register(ak8963::CNTL, mag_mode_map[MagMode::POWER_DOWN].byte);
  Delay(10);

  WriteAK8963Register(ak8963::CNTL,
                      mag_mode_map[MagMode::FUSE_ROM_ACCESS].byte);
  Delay(10);

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
  const std::string msg = "MPU9250" + valid_str +
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
  //     CalibrateGyroscope(read_gyro_data, sensor_specs_map[GyroType]);
  // sensor_specs_map[AccelType] =
  //     CalibrateAccelerometer(read_accel_data, sensor_specs_map[AccelType]);
  sensor_specs_map[MagType] =
      CalibrateMagnetometer(read_mag_data, sensor_specs_map[MagType]);
}

void Mpu9250::Update()
{
  const auto raw = ReadRawData();
  ImuData imu = ApplySensorSpecs(raw);
  imu.tait_bryan.data = mpu::EstimateRPY(imu.accel.data);
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
      full_bits[i] = To16Bit(data[i * 2], data[i * 2 + 1]);
    }
    else
    {
      full_bits[i] = To16Bit(data[i * 2 + 1], data[i * 2]);
    }
  }
  full_bits[10] = data[20];
  return full_bits;
}

SensorRawData Mpu9250::FullBitsToRawData(const std::vector<int16_t>& full_bits)
{
  assert(full_bits.size() == 11);
  SensorRawData raw;
  raw.accel = Vec3From16Bits(full_bits.begin() + 0);
  raw.gyro = Vec3From16Bits(full_bits.begin() + 4);
  raw.mag = Vec3From16Bits(full_bits.begin() + 7);
  raw.temp = static_cast<float>(full_bits[3]);
  raw.mag_over_flow = full_bits[10] == 0x08_uc;  // HOFS 4th bit
  return raw;
}

ImuData Mpu9250::ApplySensorSpecs(const SensorRawData& raw) const
{
  ImuData imu;
  imu.accel.data = sensor_specs_map[AccelType].Apply(raw.accel);
  imu.gyro.data = sensor_specs_map[GyroType].Apply(raw.gyro);
  imu.mag.data = sensor_specs_map[MagType].Apply(raw.mag);
  imu.temp.value = sensor_specs_map[TempType].Apply(raw.temp);
  if (raw.mag_over_flow)
  {
    node_->LogDebug("detect over flow");
    // imu.mag.data = Vec3::Zero();
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
  Delay(1);
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
  WriteRegister(reg, SetFlags(current, mask, byte));
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
}  // namespace mpu