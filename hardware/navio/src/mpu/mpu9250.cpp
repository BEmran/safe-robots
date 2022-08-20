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

void PrintConfig(const Config& cfg)
{
  std::cout << "\nAccelerometer:"
            << " Bandwidth " << accel_bw_map[cfg.accel_bw].name
            << ", Full Scale: " << accel_scale_map[cfg.accel_scale].name
            << "\nGyroscope:"
            << " Bandwidth " << gyro_bw_map[cfg.gyro_bw].name
            << ", Full Scale: " << gyro_scale_map[cfg.gyro_scale].name
            << "\nMagnetometer:"
            << " Mode " << mag_mode_map[cfg.mag_mode].name
            << ", Full Scale: " << mag_scale_map[cfg.mag_scale].name
            << "\nSample rate devisor: "
            << static_cast<int>(cfg.sample_rate_divisor) << std::endl;
}
}  // namespace

Mpu9250::Mpu9250(const Config& config, std::unique_ptr<SPI> comm,
                 const bool debug)
  : ImuSensorModule(ImuType, SensorName, debug)
  , config_(config)
  , comm_{std::move(comm)}
{
  sensor_specs_map[AccelType] =
      CreateSensorSpecs(accel_scale_map[config.accel_scale].value, GRAVITY);
  sensor_specs_map[GyroType] =
      CreateSensorSpecs(gyro_scale_map[config.gyro_scale].value, PI / 180.0F);
  sensor_specs_map[MagType] =
      CreateSensorSpecs(mag_scale_map[config.mag_scale].value, 1.0F);
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
  WriteRegister(mpu9250::SMPLRT_DIV, config_.sample_rate_divisor);

  InitializeGyro();
  InitializeAccel();
  ConfigureI2C();
  InitializeMag();
  ExtractMagnetometerSensitivityAdjustmentValues();

  ValidateConfiguration();
}

bool Mpu9250::Test()
{
  return false;
}

void Mpu9250::InitializeAccel() const
{
  // Set accelerometer full-scale range configuration
  const auto conf1 = ReadRegister(mpu9250::ACCEL_CONFIG);
  const auto scale = accel_scale_map[config_.accel_scale].byte;
  // Clear FS scale (bits [4:3])
  const auto mask1 = 0xE7_uc;
  // Write new ACCEL_CONFIG register value
  WriteRegister(mpu9250::ACCEL_CONFIG, SetFlags(conf1, mask1, scale));

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is 1.03
  // kHz
  // get current ACCEL_CONFIG2 register value
  const auto conf2 = ReadRegister(mpu9250::ACCEL_CONFIG2);
  const auto bw = accel_bw_map[config_.accel_bw].byte;
  // Clear b_fChoice (bit 3) and A_DLPFG (bits [2:0])
  const auto mask2 = 0xF0_uc;
  // Write new ACCEL_CONFIG2 register value
  WriteRegister(mpu9250::ACCEL_CONFIG2, SetFlags(conf2, mask2, bw));
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
  const auto conf = ReadRegister(mpu9250::GYRO_CONFIG);
  const auto bw = gyro_bw_map[config_.gyro_bw].byte;
  // Clear DLPF bits [2:0]
  const auto mask = 0xF8_uc;
  WriteRegister(mpu9250::CONFIG, SetFlags(conf, mask, bw));

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  // get current GYRO_CONFIG register value
  const auto gconf = ReadRegister(mpu9250::GYRO_CONFIG);
  const auto scale = gyro_scale_map[config_.gyro_scale].byte;
  // Clear b_fChoice bits [1:0] and AFS bits [4:3]
  const auto gmask = 0xE4_uc;
  WriteRegister(mpu9250::GYRO_CONFIG, SetFlags(gconf, gmask, scale));
}

void Mpu9250::InitializeMag() const
{
  // printf("write CNTL\n");
  // WriteAK8963Register(ak8963::CNTL, MagMode::POWER_DOWN);
  // Delay(10);

  // Configure the magnetometer for continuous read and highest resolution
  // set Scale bit 4 to 1 or (0) to enable 16 or (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // Set magnetometer data resolution and sample ODR+
  const auto scale = mag_scale_map[config_.mag_scale].byte;
  const auto mode = mag_mode_map[config_.mag_mode].byte;
  const auto res = static_cast<uint8_t>(scale | mode);
  printf("scale %d mode %d or %d\n", scale, mode, res);
  WriteAK8963Register(ak8963::CNTL, res);
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
    sensitivity_calibration_[i] =
        static_cast<float>(asa_values[i] - 128) / 256.0F + 1.0F;
    std::cout << sensitivity_calibration_[i] << std::endl;
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
    std::cerr << "Error: Accelerometer fChoice is disabled" << std::endl;
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
    std::cerr << "Error: Gyroscope fChoice is disabled" << std::endl;
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

  if (valid)
  {
    std::cout << "MPU9250 is successfully initialized! configuration is set to";
  }
  else
  {
    std::cerr << "MPU9250 Failed to initialized properly! Actual configuration "
                 "is set to";
  }
  PrintConfig(actual_cfg);

  return {valid, actual_cfg};
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
  const auto over_flow = raw_data[20] == 0x08_uc;  // HOFS 4th bit
  imu.mag = ExtractMagnetometer({full_bits[7], full_bits[8], full_bits[9]},
                                over_flow);

  return imu;
}

std::vector<int16_t> Mpu9250::ExtractFullBits(const std::vector<uint8_t>& raw)
{
  // Turn the MSB and LSB into a signed 16-bit value)
  std::vector<int16_t> full_bits(10, 0);
  for (size_t i = 0; i < full_bits.size(); i++)
  {
    if (i < 7)
    {
      full_bits[i] = To16Bit(raw[i * 2], raw[i * 2 + 1]);
    }
    else
    {
      full_bits[i] = To16Bit(raw[i * 2 + 1], raw[i * 2]);
    }
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
  temp.value = ((static_cast<float>(full_bits) - 21.F) / 333.87F) + 21.F;
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
       ak8963::I2C_ADDR | mpu9250::I2C_READ_FLAG},  // set slave 0 to the AK8963
                                                    // and set for read
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

void Mpu9250::WriteRegister(const uint8_t reg, const uint8_t data) const
{
  comm_->WriteRegister(reg, data);
}

void Mpu9250::WriteAK8963Register(const uint8_t reg, const uint8_t data) const
{
  constexpr uint8_t count = 1;
  const std::vector<std::pair<uint8_t, uint8_t>> reg_and_data{
      {mpu9250::I2C_SLV0_ADDR, ak8963::I2C_ADDR},  // set slave 0 to the AK8963
                                                   // and set for write
      {mpu9250::I2C_SLV0_REG, reg},  // set the register to the desired AK8963
                                     // sub address
      {mpu9250::I2C_SLV0_DO, data},  // store the data for write
      {mpu9250::I2C_SLV0_CTRL, mpu9250::I2C_SLV0_EN | count},  // enable I2C and
                                                               // send 1 byte
  };
  comm_->WriteRegisters(reg_and_data);
}
}  // namespace mpu