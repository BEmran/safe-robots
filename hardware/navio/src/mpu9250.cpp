/*
Written by Qiyong Mu (kylongmu@msn.com)
Adapted for Raspberry Pi by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
*/

#include "navio/mpu9250.h"
#include <unistd.h>  // usleep
#include <string>
#include <algorithm>

namespace
{
constexpr float G_SI = 9.80665F;
constexpr float PI = 3.14159F;

// MPU9250 registers
constexpr uint8_t MPUREG_XG_OFFS_TC = 0x00;
constexpr uint8_t MPUREG_YG_OFFS_TC = 0x01;
constexpr uint8_t MPUREG_ZG_OFFS_TC = 0x02;
constexpr uint8_t MPUREG_X_FINE_GAIN = 0x03;
constexpr uint8_t MPUREG_Y_FINE_GAIN = 0x04;
constexpr uint8_t MPUREG_Z_FINE_GAIN = 0x05;
constexpr uint8_t MPUREG_XA_OFFS_H = 0x06;
constexpr uint8_t MPUREG_XA_OFFS_L = 0x07;
constexpr uint8_t MPUREG_YA_OFFS_H = 0x08;
constexpr uint8_t MPUREG_YA_OFFS_L = 0x09;
constexpr uint8_t MPUREG_ZA_OFFS_H = 0x0A;
constexpr uint8_t MPUREG_ZA_OFFS_L = 0x0B;
constexpr uint8_t MPUREG_PRODUCT_ID = 0x0C;
constexpr uint8_t MPUREG_SELF_TEST_X = 0x0D;
constexpr uint8_t MPUREG_SELF_TEST_Y = 0x0E;
constexpr uint8_t MPUREG_SELF_TEST_Z = 0x0F;
constexpr uint8_t MPUREG_SELF_TEST_A = 0x10;
constexpr uint8_t MPUREG_XG_OFFS_USRH = 0x13;
constexpr uint8_t MPUREG_XG_OFFS_USRL = 0x14;
constexpr uint8_t MPUREG_YG_OFFS_USRH = 0x15;
constexpr uint8_t MPUREG_YG_OFFS_USRL = 0x16;
constexpr uint8_t MPUREG_ZG_OFFS_USRH = 0x17;
constexpr uint8_t MPUREG_ZG_OFFS_USRL = 0x18;
constexpr uint8_t MPUREG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPUREG_CONFIG = 0x1A;
constexpr uint8_t MPUREG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPUREG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPUREG_ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t MPUREG_LP_ACCEL_ODR = 0x1E;
constexpr uint8_t MPUREG_MOT_THR = 0x1F;
constexpr uint8_t MPUREG_FIFO_EN = 0x23;
constexpr uint8_t MPUREG_I2C_MST_CTRL = 0x24;
constexpr uint8_t MPUREG_I2C_SLV0_ADDR = 0x25;
constexpr uint8_t MPUREG_I2C_SLV0_REG = 0x26;
constexpr uint8_t MPUREG_I2C_SLV0_CTRL = 0x27;
constexpr uint8_t MPUREG_I2C_SLV1_ADDR = 0x28;
constexpr uint8_t MPUREG_I2C_SLV1_REG = 0x29;
constexpr uint8_t MPUREG_I2C_SLV1_CTRL = 0x2A;
constexpr uint8_t MPUREG_I2C_SLV2_ADDR = 0x2B;
constexpr uint8_t MPUREG_I2C_SLV2_REG = 0x2C;
constexpr uint8_t MPUREG_I2C_SLV2_CTRL = 0x2D;
constexpr uint8_t MPUREG_I2C_SLV3_ADDR = 0x2E;
constexpr uint8_t MPUREG_I2C_SLV3_REG = 0x2F;
constexpr uint8_t MPUREG_I2C_SLV3_CTRL = 0x30;
constexpr uint8_t MPUREG_I2C_SLV4_ADDR = 0x31;
constexpr uint8_t MPUREG_I2C_SLV4_REG = 0x32;
constexpr uint8_t MPUREG_I2C_SLV4_DO = 0x33;
constexpr uint8_t MPUREG_I2C_SLV4_CTRL = 0x34;
constexpr uint8_t MPUREG_I2C_SLV4_DI = 0x35;
constexpr uint8_t MPUREG_I2C_MST_STATUS = 0x36;
constexpr uint8_t MPUREG_INT_PIN_CFG = 0x37;
constexpr uint8_t MPUREG_INT_ENABLE = 0x38;
constexpr uint8_t MPUREG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPUREG_ACCEL_XOUT_L = 0x3C;
constexpr uint8_t MPUREG_ACCEL_YOUT_H = 0x3D;
constexpr uint8_t MPUREG_ACCEL_YOUT_L = 0x3E;
constexpr uint8_t MPUREG_ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t MPUREG_ACCEL_ZOUT_L = 0x40;
constexpr uint8_t MPUREG_TEMP_OUT_H = 0x41;
constexpr uint8_t MPUREG_TEMP_OUT_L = 0x42;
constexpr uint8_t MPUREG_GYRO_XOUT_H = 0x43;
constexpr uint8_t MPUREG_GYRO_XOUT_L = 0x44;
constexpr uint8_t MPUREG_GYRO_YOUT_H = 0x45;
constexpr uint8_t MPUREG_GYRO_YOUT_L = 0x46;
constexpr uint8_t MPUREG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t MPUREG_GYRO_ZOUT_L = 0x48;
constexpr uint8_t MPUREG_EXT_SENS_DATA_00 = 0x49;
constexpr uint8_t MPUREG_EXT_SENS_DATA_01 = 0x4A;
constexpr uint8_t MPUREG_EXT_SENS_DATA_02 = 0x4B;
constexpr uint8_t MPUREG_EXT_SENS_DATA_03 = 0x4C;
constexpr uint8_t MPUREG_EXT_SENS_DATA_04 = 0x4D;
constexpr uint8_t MPUREG_EXT_SENS_DATA_05 = 0x4E;
constexpr uint8_t MPUREG_EXT_SENS_DATA_06 = 0x4F;
constexpr uint8_t MPUREG_EXT_SENS_DATA_07 = 0x50;
constexpr uint8_t MPUREG_EXT_SENS_DATA_08 = 0x51;
constexpr uint8_t MPUREG_EXT_SENS_DATA_09 = 0x52;
constexpr uint8_t MPUREG_EXT_SENS_DATA_10 = 0x53;
constexpr uint8_t MPUREG_EXT_SENS_DATA_11 = 0x54;
constexpr uint8_t MPUREG_EXT_SENS_DATA_12 = 0x55;
constexpr uint8_t MPUREG_EXT_SENS_DATA_13 = 0x56;
constexpr uint8_t MPUREG_EXT_SENS_DATA_14 = 0x57;
constexpr uint8_t MPUREG_EXT_SENS_DATA_15 = 0x58;
constexpr uint8_t MPUREG_EXT_SENS_DATA_16 = 0x59;
constexpr uint8_t MPUREG_EXT_SENS_DATA_17 = 0x5A;
constexpr uint8_t MPUREG_EXT_SENS_DATA_18 = 0x5B;
constexpr uint8_t MPUREG_EXT_SENS_DATA_19 = 0x5C;
constexpr uint8_t MPUREG_EXT_SENS_DATA_20 = 0x5D;
constexpr uint8_t MPUREG_EXT_SENS_DATA_21 = 0x5E;
constexpr uint8_t MPUREG_EXT_SENS_DATA_22 = 0x5F;
constexpr uint8_t MPUREG_EXT_SENS_DATA_23 = 0x60;
constexpr uint8_t MPUREG_I2C_SLV0_DO = 0x63;
constexpr uint8_t MPUREG_I2C_SLV1_DO = 0x64;
constexpr uint8_t MPUREG_I2C_SLV2_DO = 0x65;
constexpr uint8_t MPUREG_I2C_SLV3_DO = 0x66;
constexpr uint8_t MPUREG_I2C_MST_DELAY_CTRL = 0x67;
constexpr uint8_t MPUREG_SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t MPUREG_MOT_DETECT_CTRL = 0x69;
constexpr uint8_t MPUREG_USER_CTRL = 0x6A;
constexpr uint8_t MPUREG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPUREG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t MPUREG_BANK_SEL = 0x6D;
constexpr uint8_t MPUREG_MEM_START_ADDR = 0x6E;
constexpr uint8_t MPUREG_MEM_R_W = 0x6F;
constexpr uint8_t MPUREG_DMP_CFG_1 = 0x70;
constexpr uint8_t MPUREG_DMP_CFG_2 = 0x71;
constexpr uint8_t MPUREG_FIFO_COUNTH = 0x72;
constexpr uint8_t MPUREG_FIFO_COUNTL = 0x73;
constexpr uint8_t MPUREG_FIFO_R_W = 0x74;
constexpr uint8_t MPUREG_WHOAMI = 0x75;
constexpr uint8_t MPUREG_XA_OFFSET_H = 0x77;
constexpr uint8_t MPUREG_XA_OFFSET_L = 0x78;
constexpr uint8_t MPUREG_YA_OFFSET_H = 0x7A;
constexpr uint8_t MPUREG_YA_OFFSET_L = 0x7B;
constexpr uint8_t MPUREG_ZA_OFFSET_H = 0x7D;
constexpr uint8_t MPUREG_ZA_OFFSET_L = 0x7E;

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

constexpr uint8_t AK8963_I2C_ADDR = 0x0c;  // should return 0x18
constexpr uint8_t AK8963_Device_ID = 0x48;

// Read-only Reg
constexpr uint8_t AK8963_WIA = 0x00;
constexpr uint8_t AK8963_INFO = 0x01;
constexpr uint8_t AK8963_ST1 = 0x02;
constexpr uint8_t AK8963_HXL = 0x03;
constexpr uint8_t AK8963_HXH = 0x04;
constexpr uint8_t AK8963_HYL = 0x05;
constexpr uint8_t AK8963_HYH = 0x06;
constexpr uint8_t AK8963_HZL = 0x07;
constexpr uint8_t AK8963_HZH = 0x08;
constexpr uint8_t AK8963_ST2 = 0x09;

// Write/Read Reg
constexpr uint8_t AK8963_CNTL1 = 0x0A;
constexpr uint8_t AK8963_CNTL2 = 0x0B;
constexpr uint8_t AK8963_ASTC = 0x0C;
constexpr uint8_t AK8963_TS1 = 0x0D;
constexpr uint8_t AK8963_TS2 = 0x0E;
constexpr uint8_t AK8963_I2CDIS = 0x0F;

// Read-only Reg ( ROM )
constexpr uint8_t AK8963_ASAX = 0x10;
constexpr uint8_t AK8963_ASAY = 0x11;
constexpr uint8_t AK8963_ASAZ = 0x12;
}  // namespace

constexpr const char* SPI_PATH = "/dev/spidev0.1";
//-----------------------------------------------------------------------------------------------

MPU9250::MPU9250(const bool debug)
  : core::sensors::ImuSensorModule("mpu", debug)
  , spidev_{std::make_unique<SPIdev>(SPI_PATH, debug)}
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write MPU9250 registers over SPI
-----------------------------------------------------------------------------------------------*/

uint8_t MPU9250::WriteReg(const uint8_t addr, const uint8_t data)
{
  uint8_t tx[2] = {addr, data};
  uint8_t rx[2] = {0};

  spidev_->Transfer(tx, rx, 2);

  return rx[1];
}

//-----------------------------------------------------------------------------------------------

uint8_t MPU9250::ReadReg(const uint8_t addr)
{
  return WriteReg(addr | READ_FLAG, 0x00);
}

//-----------------------------------------------------------------------------------------------

std::vector<uint8_t> MPU9250::ReadRegs(const uint8_t addr,
                                       const uint32_t length)
{
  uint8_t tx[255] = {0};
  uint8_t rx[255] = {0};

  tx[0] = addr | READ_FLAG;

  spidev_->Transfer(tx, rx, length + 1U);
  usleep(50);

  std::vector<uint8_t> buf(rx + 1, rx + length + 1);
  return buf;
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and MPU9250 are working correctly.
returns true if mpu9250 answers
-----------------------------------------------------------------------------------------------*/

bool MPU9250::Probe()
{
  uint8_t response_xg = ReadReg(MPUREG_WHOAMI | READ_FLAG);

  WriteReg(MPUREG_USER_CTRL, 0x20);  // I2C Master mode
  WriteReg(MPUREG_I2C_MST_CTRL,
           0x0D);  // I2C configuration multi-master  IIC 400KHz
  WriteReg(MPUREG_I2C_SLV0_ADDR,
           AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963
                                          // and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG,
           AK8963_WIA);  // I2C slave 0 register address from where to begin
                         // data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);  // Read 1 byte from the magnetometer
  usleep(10000);
  uint8_t response_m = ReadReg(MPUREG_EXT_SENS_DATA_00);

  return (response_xg == 0x71 && response_m == 0x48);
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup for initialize settings of sensor
low pass filter suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ
BITS_DLPF_CFG_5HZ
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/

constexpr uint8_t MPU_INIT_REG_NUM = 16;

void MPU9250::Initialize()
{
  uint8_t i = 0;
  uint8_t mpu_init_data[MPU_INIT_REG_NUM][2] = {
      //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it
      // seems to corrupt initialisation of AK8963
      {0x01, MPUREG_PWR_MGMT_1},      // Clock Source
      {0x00, MPUREG_PWR_MGMT_2},      // Enable Acc & Gyro
      {0x00, MPUREG_CONFIG},          // Use DLPF set Gyroscope bandwidth 184Hz,
                                      // temperature bandwidth 188Hz
      {0x18, MPUREG_GYRO_CONFIG},     // +-2000dps
      {3 << 3, MPUREG_ACCEL_CONFIG},  // +-16G
      {0x08, MPUREG_ACCEL_CONFIG_2},  // Set Acc Data Rates, Enable Acc LPF ,
                                      // Bandwidth 184Hz
      {0x30, MPUREG_INT_PIN_CFG},     //
      //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
      //{0x20, MPUREG_USER_CTRL},      // Enable AUX
      {0x20, MPUREG_USER_CTRL},     // I2C Master mode
      {0x0D, MPUREG_I2C_MST_CTRL},  //  I2C configuration multi-master  IIC
                                    //  400KHz

      {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of
                                                // AK8963 and set for write.
      //{0x09, MPUREG_I2C_SLV4_CTRL},
      //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

      {AK8963_CNTL2, MPUREG_I2C_SLV0_REG},  // I2C slave 0 register address from
                                            // where to begin data transfer
      {0x01, MPUREG_I2C_SLV0_DO},           // Reset AK8963
      {0x81, MPUREG_I2C_SLV0_CTRL},         // Enable I2C and set 1 byte

      {AK8963_CNTL1, MPUREG_I2C_SLV0_REG},  // I2C slave 0 register address from
                                            // where to begin data transfer
      {0x12, MPUREG_I2C_SLV0_DO},   // Register value to continuous measurement
                                    // in 16bit
      {0x81, MPUREG_I2C_SLV0_CTRL}  // Enable I2C and set 1 byte

  };

  SetAccelerometerScale(BITS_FS_16G);
  SetGyroScale(BITS_FS_2000DPS);

  for (i = 0; i < MPU_INIT_REG_NUM; i++)
  {
    WriteReg(mpu_init_data[i][1], mpu_init_data[i][0]);
    usleep(100000);  // I2C must slow down the write speed, otherwise it won't
                     // work
  }

  CalibMagnetometer();
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right
range for the accelerometers. Suitable ranges are: BITS_FS_2G BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/

uint32_t MPU9250::SetAccelerometerScale(const uint8_t scale)
{
  WriteReg(MPUREG_ACCEL_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_2G:
      acc_divider_ = 16384.0F;
      break;
    case BITS_FS_4G:
      acc_divider_ = 8192.0F;
      break;
    case BITS_FS_8G:
      acc_divider_ = 4096.0F;
      break;
    case BITS_FS_16G:
      acc_divider_ = 2048.0F;
      break;
    default:
      printf("Error: undefined accelerometer scale");
  }

  const auto temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
  uint32_t returned_scale = 0U;
  switch (temp_scale)
  {
    case BITS_FS_2G:
      returned_scale = 2U;
      break;
    case BITS_FS_4G:
      returned_scale = 4U;
      break;
    case BITS_FS_8G:
      returned_scale = 8U;
      break;
    case BITS_FS_16G:
      returned_scale = 16U;
      break;
    default:
      printf("Error: undefined accelerometer temp scale (%u)\n", returned_scale);
  }
  printf("SetAccelerometerScale %u -> %u acc_divider %f\n", scale, returned_scale, acc_divider_);
  return returned_scale;
}

/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right
range for the gyroscopes. Suitable ranges are: BITS_FS_250DPS BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/

uint32_t MPU9250::SetGyroScale(const uint8_t scale)
{
  WriteReg(MPUREG_GYRO_CONFIG, scale);
  switch (scale)
  {
    case BITS_FS_250DPS:
      gyro_divider_ = 131.0F;
      break;
    case BITS_FS_500DPS:
      gyro_divider_ = 65.5F;
      break;
    case BITS_FS_1000DPS:
      gyro_divider_ = 32.8F;
      break;
    case BITS_FS_2000DPS:
      gyro_divider_ = 16.4F;
      break;
    default:
      printf("Error: undefined gyro scale");
  }

  uint8_t temp_scale = WriteReg(MPUREG_GYRO_CONFIG | READ_FLAG, 0x00);
  uint32_t returned_scale = 1U;
  switch (temp_scale)
  {
    case BITS_FS_250DPS:
      returned_scale = 250U;
      break;
    case BITS_FS_500DPS:
      returned_scale = 500U;
      break;
    case BITS_FS_1000DPS:
      returned_scale = 1000U;
      break;
    case BITS_FS_2000DPS:
      returned_scale = 2000U;
      break;
    default:
      printf("Error: undefined returned gyro scale (%u)\n", returned_scale);
  }
  printf("SetGyroScale %u -> %u gyro_divider_ %f\n", scale, returned_scale, gyro_divider_);
  return returned_scale;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected
axis: 0 -> X axis 1 -> Y axis 2 -> Z axis returns Factory Trim value
-----------------------------------------------------------------------------------------------*/

void MPU9250::CalibAccelerometer()
{
  // read current acc scale
  const auto temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
  SetAccelerometerScale(BITS_FS_8G);
  // ENABLE SELF TEST need modify
  // temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

  const auto response = ReadRegs(MPUREG_SELF_TEST_X, 4);
  calib_data[0] =
      ((response[0] & 11100000) >> 3) | ((response[3] & 00110000) >> 4);
  calib_data[1] =
      ((response[1] & 11100000) >> 3) | ((response[3] & 00001100) >> 2);
  calib_data[2] = ((response[2] & 11100000) >> 3) | ((response[3] & 00000011));

  SetAccelerometerScale(temp_scale);
}

//-----------------------------------------------------------------------------------------------

void MPU9250::CalibMagnetometer()
{
  WriteReg(MPUREG_I2C_SLV0_ADDR,
           AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963
                                          // and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG,
           AK8963_ASAX);  // I2C slave 0 register address from where to begin
                          // data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);  // Read 3 bytes from the magnetometer

  // WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
  usleep(10000);
  // response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01 | READ_FLAG, 0x00);    //Read
  // I2C
  const auto response = ReadRegs(MPUREG_EXT_SENS_DATA_00, 3);

  // response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
  printf("magnetometer_ASA: ");
  for (size_t i = 0; i < response.size(); i++)
  {
    const auto data = static_cast<float>(response[i]);
    magnetometer_asa_[i] =
        ((data - 128.0F) / 256.0F + 1) * Magnetometer_Sensitivity_Scale_Factor;
    printf("\t%f", magnetometer_asa_[i]);
  }
  printf("\n");
}

//-----------------------------------------------------------------------------------------------

void MPU9250::Update()
{
  RequestImu();
  const auto response = ReadRegs(MPUREG_ACCEL_XOUT_H, 21);
  const auto data = ExtractData(response);
  SetData(data);
}

void MPU9250::RequestImu()
{
  // Send I2C command at first
  WriteReg(MPUREG_I2C_SLV0_ADDR,
           AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963
                                          // and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG,
           AK8963_HXL);  // I2C slave 0 register address from where to begin
                         // data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);  // Read 7 bytes from the magnetometer
  // must start your read from AK8963A register 0x03 and read seven bytes so
  // that upon read of ST2 register 0x09 the AK8963A will unlatch the data
  // registers for the next measurement.
}

uint16_t BitDataFromResponse(const std::vector<uint8_t>& response,
                             const uint idx)
{
  const uint h_idx = idx * 2U;
  const uint l_idx = h_idx + 1U;
  if (response.size() < l_idx)
  {
    printf("Error: worng index (%u)\n", idx);
    return 0;
  }
  const int16_t high = response[h_idx] << 8U;
  const int16_t low = response[l_idx];
  return high | low;
}

double MPU9250::ExtractTempreture(const std::vector<uint8_t>& response)
{
  const auto temp = BitDataFromResponse(response, 3U);
  const auto celcious = ((temp - 21) / 333.87) + 21;
  return celcious;
}

core::utils::Vec3
MPU9250::ExtractAccelerometer(const std::vector<uint8_t>& response) const
{
  core::utils::Vec3 accel = core::utils::Vec3::Zero();
  for (int i = 0; i < 3; i++)
  {
    const uint idx = static_cast<uint>(i);
    const auto bit_data =
        static_cast<float>(BitDataFromResponse(response, idx));
    accel[i] = G_SI * bit_data / acc_divider_;
  }
  return accel;
}

core::utils::Vec3
MPU9250::ExtractGyroscope(const std::vector<uint8_t>& response) const
{
  core::utils::Vec3 gyro = core::utils::Vec3::Zero();
  for (int i = 4; i < 7; i++)
  {
    const uint idx = static_cast<uint>(i);
    const auto bit_data =
        static_cast<float>(BitDataFromResponse(response, idx));
    gyro[i - 4] = (PI / 180) * bit_data / gyro_divider_;
  }
  return gyro;
}

core::utils::Vec3
MPU9250::ExtractMagnetometer(const std::vector<uint8_t>& response) const
{
  core::utils::Vec3 mag = core::utils::Vec3::Zero();
  for (int i = 7; i < 10; i++)
  {
    const uint idx = static_cast<uint>(i);
    const auto bit_data =
        static_cast<float>(BitDataFromResponse(response, idx));
    mag[i - 7] = bit_data * magnetometer_asa_[i - 7];
  }
  return mag;
}

core::utils::ImuData
MPU9250::ExtractData(const std::vector<uint8_t>& response) const
{
  core::utils::ImuData data;
  data.temp = ExtractTempreture(response);
  data.accel = ExtractAccelerometer(response);
  data.gyro = ExtractGyroscope(response);
  data.mag = ExtractMagnetometer(response);
  return data;
}