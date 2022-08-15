#include <mpu/mpu_gyro.hpp>
#include <mpu/spi.hpp>
#include <numeric>

namespace mpu
{
constexpr uint16_t GyroScales[4] = {250, 500, 1000, 2000};

MpuGyro::MpuGyro(const GyroConfig& config, const bool debug)
  : SensorModuleGyroscope(GyroscopeType, GyroscopeSensorName, debug)
  , config_(config)
{
  GyroResolution();
  GyroSensitivity();
  printf("scale %u sensitivity_ %f resolution_ %f\n", Scale(), sensitivity_, resolution_);
}

void MpuGyro::GyroSensitivity()
{
  sensitivity_ = 32768 / GyroScales[Scale()];
}
void MpuGyro::GyroResolution()
{
  resolution_ = GyroScales[Scale()] / 32768.f;
}

void MpuGyro::Reset()
{
}

void MpuGyro::Initialize()
{
  // wake up device
  // GetSpi()->WriteRegister(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6),
  // enable all sensors Clear sleep mode bit (6), enable all sensors
  GetSpi()->WriteRegister(PWR_MGMT_1, 0x80);
  delay(100);  // Wait for all registers to reset

  // get stable time source. Auto select clock source to be PLL  gyroscope
  // reference if ready else
  GetSpi()->WriteRegister(PWR_MGMT_1, 0x01);
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively; minimum delay time for this setting is 5.9 ms, which means
  // sensor fusion update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8
  // kHz, or 1 kHz

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  GetSpi()->WriteRegister(SMPLRT_DIV, config_.sample_rate_divisor);

  // determined inset in CONFIG above

  GetSpi()->WriteRegister(CONFIG, 0x01);  // Use DLPF set Gyroscope bandwidth
                                          // 184Hz, temperature bandwidth 188Hz

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3
  // get current GYRO_CONFIG register value
  uint8_t c = ReadRegister(GYRO_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03;    // Clear Fchoice bits [1:0]
  c = c & ~0x18;    // Clear AFS bits [4:3]
  c = c | Scale() << 3;  // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits
  // 1:0 of GYRO_CONFIG
  // Write new GYRO_CONFIG value to register
  printf("Config %u\n", c);
  GetSpi()->WriteRegister(GYRO_CONFIG, c);

  // // Set accelerometer full-scale range configuration
  // c = readMPURegister(ACCEL_CONFIG);  // get current ACCEL_CONFIG register
  // value
  // // c = c & ~0xE0; // Clear self-test bits [7:5]
  // c = c & ~0x18;        // Clear AFS bits [4:3]
  // c = c | ascale << 3;  // Set full scale range for the accelerometer
  // GetSpi()->WriteRegister(ACCEL_CONFIG,
  //                         c);  // Write new ACCEL_CONFIG register value

  // // Set accelerometer sample rate configuration
  // // It is possible to get a 4 kHz sample rate from the accelerometer by
  // // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth
  // is 1.13
  // // kHz
  // c = readMPURegister(ACCEL_CONFIG2);  // get current ACCEL_CONFIG2 register
  //                                      // value
  // c = c & ~0x0F;  // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  // c = c | 0x03;   // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // GetSpi()->WriteRegister(ACCEL_CONFIG2,
  //                         c);  // Write new ACCEL_CONFIG2 register value

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // enable master mode
  GetSpi()->WriteRegister(USER_CTRL, I2C_MST_EN);
  // I2C configuration multi-master  IIC 400KHz
  GetSpi()->WriteRegister(I2C_MST_CTRL, 0x0D);

  // GetSpi()->WriteRegister(INT_ENABLE, 0x01);  // Enable data ready (bit 0)
  // interrupt
  delay(100);
}

uint8_t MpuGyro::Scale() const
{
  return static_cast<uint8_t>(config_.scale);
}

bool MpuGyro::Probe()
{
  // check AK8963 WHO AM I register, expected value is 0x48 (decimal 113)
  printf("Gyro Whom %u\n",  ReadRegister(mpu::WHO_AM_I));
  return ReadRegister(mpu::WHO_AM_I) == 0x71;
}

bool MpuGyro::Test()
{
  return false;
}

void MpuGyro::Update()
{
  const auto data = ReadGyroscope();
  SetData(data);
}

uint8_t MpuGyro::ReadRegister(uint8_t reg) const
{
  uint8_t buffer[1] = {0};
  GetSpi()->ReadRegisters(reg, 1, buffer);
  return buffer[0];
}

GyroData MpuGyro::ReadGyroscope() const
{
  std::array<int16_t, 3> gyro_full_bits = ReadGyroFullBits();

  GyroData gyro;
  for (size_t i = 0; i < gyro_full_bits.size(); i++)
  {
    gyro.data[i] = (PI / 180.0F) * (float)gyro_full_bits[i] * resolution_;
  }
  printf("resolution_:%f\n", resolution_);
  printf("gyroCount x:%d\t y:%d\t z:%d\n", gyro_full_bits[0], gyro_full_bits[1], gyro_full_bits[2]);

  return gyro;
}

std::array<int16_t, 3> MpuGyro::ReadGyroFullBits() const
{
  std::array<int16_t, 3> values = {0, 0, 0};
  // Read the six raw data registers sequentially into data array
  uint8_t raw_data[6];
  GetSpi()->ReadRegisters(mpu::GYRO_XOUT_H, 6, &raw_data[0]);
  // Turn the MSB and LSB into a signed 16-bit value
  for (size_t i = 0; i < values.size(); i++)
  {
    values[i] = To16Bit(raw_data[i * 2], raw_data[i * 2 + 1]);
  }
  return values;
}

void MpuGyro::Calibrate(void)
{
}

}  // namespace mpu