#include <mpu/AK8963.hpp>
#include <mpu/spi.hpp>
#include <numeric>

namespace mpu
{
namespace ak8963
{
// Magnetometer legister map
constexpr uint8_t WHO_AM_I = 0x00;
constexpr uint8_t INFO = 0x01;
constexpr uint8_t ST1 = 0x02;
constexpr uint8_t XOUT_L = 0x03;
constexpr uint8_t XOUT_H = 0x04;
constexpr uint8_t YOUT_L = 0x05;
constexpr uint8_t YOUT_H = 0x06;
constexpr uint8_t ZOUT_L = 0x07;
constexpr uint8_t ZOUT_H = 0x08;
constexpr uint8_t ST2 = 0x09;
constexpr uint8_t CNTL = 0x0A;
constexpr uint8_t ASTC = 0x0C;
constexpr uint8_t I2CDIS = 0x0F;
constexpr uint8_t ASAX = 0x10;
constexpr uint8_t ASAY = 0x11;
constexpr uint8_t ASAZ = 0x12;
}  // namespace ak8963

AK8963::AK8963(const AK8963Config& config, const bool debug)
  : SensorModuleMagnetometer(MagnetometerType, MagnetometerSensorName, debug)
  , config_(config)
{
  MagRes();
}

void AK8963::MagRes()
{
  switch (config_.scale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (bit: 0) and 16 bit resolution (bit: 1)
    // We multiply by 10 to convert microteslas to milligauss
    // mag_res_ = 18 * Maximum Magnetic flux density [µT] / half bit resolution
    case MagScale::MFS_16BITS:
      resolution_ = 10. * 4912.0 / 32760.0;
      break;
    case MagScale::MFS_14BITS:
    default:
      resolution_ = 10. * 4912.0 / 8190.0;
  }
}

void AK8963::Reset()
{
  printf("USER_CTRL: disable internal\n");
  GetSpi()->WriteRegister(mpu::USER_CTRL, 0);  // disable internal I2C bus

  // reset device
  printf("PWR_MGMT_1\n");
  GetSpi()->WriteRegister(mpu::PWR_MGMT_1, 0x80);  // Set bit 7 to reset MPU9250
  delay(10);

  printf("USER_CTRL:\n");
  GetSpi()->WriteRegister(mpu::USER_CTRL, I2C_MST_EN);  // re-enable internal I2C bus

  printf("I2C_MST_CTRL:\n");
  GetSpi()->WriteRegister(mpu::I2C_MST_CTRL, 0x0D); // I2C configuration multi-master  IIC 400KHz
  
  delay(100);  // Wait for all registers to reset
}

void AK8963::Initialize()
{
  Reset();
  ExtractSensitivityAdjustmentValues();
  ConfigureScaleAndMode();
}

void AK8963::ExtractSensitivityAdjustmentValues()
{
  // Power down magnetometer
  WriteRegister(ak8963::CNTL, 0x00);
  delay(10);
  // Enter Fuse ROM access mode
  WriteRegister(ak8963::CNTL, 0x0F);
  delay(10);
  // Read the x-, y-, and z-axis calibration values
  uint8_t asa_values[3];
  ReadRegisters(ak8963::ASAX, 3, &asa_values[0]);
  // Return xyz-axis sensitivity adjustment values
  for (size_t i = 0; i < 3; i++)
  {
    sensitivity_calibration_[i] = (float)(asa_values[i] - 128) / 256.0f + 1.0f;
  }
  // debug
  printf("sensitivity x:%f\t y:%f\t z:%f\n", sensitivity_calibration_[0],
         sensitivity_calibration_[1], sensitivity_calibration_[2]);
}

void AK8963::ConfigureScaleAndMode() const
{
  WriteRegister(ak8963::CNTL, POWER_DOWN_MODE);
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Scale bit 4 to 1 or (0) to enable 16 or (14) bit resolution in CNTL
  // register, and enable continuous mode data acquisition Mode (bits [3:0]),
  // 0010 for 8 Hz and 0110 for 100 Hz sample rates
  // Set magnetometer data resolution and sample ODR+
  printf("scale %u\n", Scale());
  printf("Mode %u\n", Mode());
  WriteRegister(ak8963::CNTL, Scale() | Mode());
  delay(10);
}

uint8_t AK8963::Mode() const
{
  return static_cast<uint16_t>(config_.mode);
}

uint8_t AK8963::Scale() const
{
  return static_cast<uint16_t>(config_.scale);
}

bool AK8963::Probe()
{
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
  return ReadRegister(ak8963::WHO_AM_I) != 0x48;
}

bool AK8963::Test()
{
  return false;
}

void AK8963::Update()
{
  const auto mag_data = ReadMagnetometer();
  SetData(mag_data);
}

// void AK8963::Calibrate()
// {
// }

void AK8963::WriteRegister(const uint8_t reg, const uint8_t data) const
{
  const uint8_t count = 1;
  // set slave 0 to the AK8963 and set for write
  GetSpi()->WriteRegister(mpu::I2C_SLV0_ADDR, mpu::AK8963_ADDRESS);
  // set the register to the desired AK8963 sub address
  GetSpi()->WriteRegister(mpu::I2C_SLV0_REG, reg);
  // store the data for write
  GetSpi()->WriteRegister(mpu::I2C_SLV0_DO, data);
  // enable I2C and send 1 byte
  GetSpi()->WriteRegister(mpu::I2C_SLV0_CTRL, mpu::I2C_SLV0_EN | count);
}

uint8_t AK8963::ReadRegister(uint8_t reg) const
{
  uint8_t buffer[1] = {0};
  ReadRegisters(reg, 1, buffer);
  return buffer[0];
}

void AK8963::ReadRegisters(const uint8_t reg, const uint8_t count,
                           uint8_t* dest) const
{
  // set slave 0 to the AK8963 and set for read
  GetSpi()->WriteRegister(mpu::I2C_SLV0_ADDR,
                       mpu::AK8963_ADDRESS | mpu::I2C_READ_FLAG);
  // set the register to the desired AK8963 sub address
  GetSpi()->WriteRegister(mpu::I2C_SLV0_REG, reg);
  // enable I2C and request the bytes
  GetSpi()->WriteRegister(mpu::I2C_SLV0_CTRL, mpu::I2C_SLV0_EN | count);
  // takes some time for these registers to fill
  delay(10);
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
  GetSpi()->ReadRegisters(EXT_SENS_DATA_00, count, dest);
}

MagData AK8963::ReadMagnetometer() const
{
  std::array<uint16_t, 3> mag_full_bits = ReadMagFullBits();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections Get actual magnetometer value, this depends on scale being set
  MagData mag;
  for (size_t i = 0; i < mag_full_bits.size(); i++)
  {
    const float tmp =
        (float)mag_full_bits[i] * resolution_ * sensitivity_calibration_[i] -
        bias_correction_[i];
    mag.data[i] = tmp * scale_correction_[i];
  }
  // printf("mag_full_bits x:%u\t y:%u\t z:%u\n", mag_full_bits[0],
  //        mag_full_bits[1], mag_full_bits[2]);
  // printf("_mRes: %f\n", resolution_);
  // printf("_magCalibration x:%f\t y:%f\t z:%f\n", sensitivity_calibration_[0],
  //        sensitivity_calibration_[1], sensitivity_calibration_[2]);
  // printf("_magBias x:%f\t y:%f\t z:%f\n", bias_correction_[0],
  //        bias_correction_[1], bias_correction_[2]);
  // printf("_magScale x:%f\t y:%f\t z:%f\n", scale_correction_[0],
  //        scale_correction_[1], scale_correction_[2]);
  return mag;
}

std::array<uint16_t, 3> AK8963::ReadMagFullBits() const
{
  std::array<uint16_t, 3> values = {0, 0, 0};
  // Read the six raw data and ST2 registers, must read ST2 at end of data
  // acquisition
  uint8_t raw_data[7];
  ReadRegisters(ak8963::XOUT_L, 7, &raw_data[0]);

  const uint8_t sensor_over_flow = raw_data[6] & 0x08;  // HOFS 4th bit

  // Check if magnetic sensor overflow set, if not then report data
  if (sensor_over_flow)
  {
    printf("Warning: Magnetometer sensor detected data overflow \n");
  }
  else
  {
    // Data stored as little Endian
    for (size_t i = 0; i < values.size(); i++)
    {
      values[i] = To16Bit(raw_data[i * 2 + 1], raw_data[i * 2]);
    }
  }
  return values;
}

void AK8963::Calibrate(void)
{
  // at 8 Hz ODR, new mag data is available every 125 ms, at 100 Hz ODR, new mag
  // data is available every 10 ms
  const uint16_t samples =
      config_.mode == MagMode::CONTINUES_8HZ_MODE ? 128 : 1500;
  constexpr int16_t max_val = 32767;  // 2^16/2
  std::vector<uint16_t> vector_of_values(samples);
  std::array<std::vector<uint16_t>, 3> mag_all_values{vector_of_values};

  for (size_t i = 0; i < vector_of_values.size(); i++)
  {
    std::array<uint16_t, 3> mag_full_bits = ReadMagFullBits();
    printf("[%u] x:%u\t y:%u\t z:%u\n", i, mag_full_bits[0], mag_full_bits[1],
           mag_full_bits[2]);
    for (size_t j = 0; j < mag_all_values.size(); j++)
    {
      mag_all_values[j][i] = mag_full_bits[j];
    }
    config_.mode == MagMode::CONTINUES_8HZ_MODE ? delay(130) : delay(15);
  }

  for (size_t j = 0; j < mag_all_values.size(); j++)
  {
    const auto [min, max] =
        std::minmax_element(begin(mag_all_values[j]), end(mag_all_values[j]));
    printf("[%u] max: %u\t min: %u\n", j, *max, *min);
    // Get hard iron correction: get average xyz mag bias in counts
    const auto tmp = (float)(*max + *min) / 2.0;
    bias_correction_[j] = tmp * resolution_ * sensitivity_calibration_[j];
    // Get soft iron correction estimate: get average xyz axis max chord length
    // in counts
    scale_correction_[j] = (float)(*max - *min) / 2.0;
  }

  const float avg_rad =
      std::accumulate(scale_correction_.begin(), scale_correction_.end(), 0.0) /
      scale_correction_.size();
  std::for_each(scale_correction_.begin(), scale_correction_.end(),
                [avg_rad](auto& sc) { sc = avg_rad / sc; });
  // debug
  for (size_t j = 0; j < scale_correction_.size(); j++)
  {
    printf("[%u] bias: %f\t scale: %f\n", j, bias_correction_[j],
           scale_correction_[j]);
  }
}

}  // namespace mpu