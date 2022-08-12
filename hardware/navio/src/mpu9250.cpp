/*
Written by Qiyong Mu (kylongmu@msn.com)
Adapted for Raspberry Pi by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
*/

#include "navio/mpu9250.h"
#include <string>

#define G_SI 9.80665
#define PI 3.14159

// MPU9250 registers
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_SELF_TEST_X 0x0D
#define MPUREG_SELF_TEST_Y 0x0E
#define MPUREG_SELF_TEST_Z 0x0F
#define MPUREG_SELF_TEST_A 0x10
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_ACCEL_CONFIG_2 0x1D
#define MPUREG_LP_ACCEL_ODR 0x1E
#define MPUREG_MOT_THR 0x1F
#define MPUREG_FIFO_EN 0x23
#define MPUREG_I2C_MST_CTRL 0x24
#define MPUREG_I2C_SLV0_ADDR 0x25
#define MPUREG_I2C_SLV0_REG 0x26
#define MPUREG_I2C_SLV0_CTRL 0x27
#define MPUREG_I2C_SLV1_ADDR 0x28
#define MPUREG_I2C_SLV1_REG 0x29
#define MPUREG_I2C_SLV1_CTRL 0x2A
#define MPUREG_I2C_SLV2_ADDR 0x2B
#define MPUREG_I2C_SLV2_REG 0x2C
#define MPUREG_I2C_SLV2_CTRL 0x2D
#define MPUREG_I2C_SLV3_ADDR 0x2E
#define MPUREG_I2C_SLV3_REG 0x2F
#define MPUREG_I2C_SLV3_CTRL 0x30
#define MPUREG_I2C_SLV4_ADDR 0x31
#define MPUREG_I2C_SLV4_REG 0x32
#define MPUREG_I2C_SLV4_DO 0x33
#define MPUREG_I2C_SLV4_CTRL 0x34
#define MPUREG_I2C_SLV4_DI 0x35
#define MPUREG_I2C_MST_STATUS 0x36
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_TEMP_OUT_H 0x41
#define MPUREG_TEMP_OUT_L 0x42
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_EXT_SENS_DATA_00 0x49
#define MPUREG_EXT_SENS_DATA_01 0x4A
#define MPUREG_EXT_SENS_DATA_02 0x4B
#define MPUREG_EXT_SENS_DATA_03 0x4C
#define MPUREG_EXT_SENS_DATA_04 0x4D
#define MPUREG_EXT_SENS_DATA_05 0x4E
#define MPUREG_EXT_SENS_DATA_06 0x4F
#define MPUREG_EXT_SENS_DATA_07 0x50
#define MPUREG_EXT_SENS_DATA_08 0x51
#define MPUREG_EXT_SENS_DATA_09 0x52
#define MPUREG_EXT_SENS_DATA_10 0x53
#define MPUREG_EXT_SENS_DATA_11 0x54
#define MPUREG_EXT_SENS_DATA_12 0x55
#define MPUREG_EXT_SENS_DATA_13 0x56
#define MPUREG_EXT_SENS_DATA_14 0x57
#define MPUREG_EXT_SENS_DATA_15 0x58
#define MPUREG_EXT_SENS_DATA_16 0x59
#define MPUREG_EXT_SENS_DATA_17 0x5A
#define MPUREG_EXT_SENS_DATA_18 0x5B
#define MPUREG_EXT_SENS_DATA_19 0x5C
#define MPUREG_EXT_SENS_DATA_20 0x5D
#define MPUREG_EXT_SENS_DATA_21 0x5E
#define MPUREG_EXT_SENS_DATA_22 0x5F
#define MPUREG_EXT_SENS_DATA_23 0x60
#define MPUREG_I2C_SLV0_DO 0x63
#define MPUREG_I2C_SLV1_DO 0x64
#define MPUREG_I2C_SLV2_DO 0x65
#define MPUREG_I2C_SLV3_DO 0x66
#define MPUREG_I2C_MST_DELAY_CTRL 0x67
#define MPUREG_SIGNAL_PATH_RESET 0x68
#define MPUREG_MOT_DETECT_CTRL 0x69
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75
#define MPUREG_XA_OFFSET_H 0x77
#define MPUREG_XA_OFFSET_L 0x78
#define MPUREG_YA_OFFSET_H 0x7A
#define MPUREG_YA_OFFSET_L 0x7B
#define MPUREG_ZA_OFFSET_H 0x7D
#define MPUREG_ZA_OFFSET_L 0x7E

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR 0x0c  // should return 0x18
#define AK8963_Device_ID 0x48

// Read-only Reg
#define AK8963_WIA 0x00
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_HXH 0x04
#define AK8963_HYL 0x05
#define AK8963_HYH 0x06
#define AK8963_HZL 0x07
#define AK8963_HZH 0x08
#define AK8963_ST2 0x09

// Write/Read Reg
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASTC 0x0C
#define AK8963_TS1 0x0D
#define AK8963_TS2 0x0E
#define AK8963_I2CDIS 0x0F

// Read-only Reg ( ROM )
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

constexpr const char* SpiPath = "/dev/spidev0.1";
//-----------------------------------------------------------------------------------------------

MPU9250::MPU9250(const bool debug)
  : core::sensors::ImuSensorModule("mpu", debug)
  , spidev_{std::make_unique<SPIdev>(SpiPath, debug)}
{
}

/*-----------------------------------------------------------------------------------------------
                                    REGISTER READ & WRITE
usage: use these methods to read and write MPU9250 registers over SPI
-----------------------------------------------------------------------------------------------*/

uint8_t MPU9250::WriteReg(uint8_t WriteAddr, uint8_t WriteData)
{
  uint8_t tx[2] = {WriteAddr, WriteData};
  uint8_t rx[2] = {0};

  spidev_->Transfer(tx, rx, 2);

  return rx[1];
}

//-----------------------------------------------------------------------------------------------

uint8_t MPU9250::ReadReg(uint8_t ReadAddr)
{
  return WriteReg(ReadAddr | READ_FLAG, 0x00);
}

//-----------------------------------------------------------------------------------------------

void MPU9250::ReadRegs(uint8_t ReadAddr, uint8_t* ReadBuf, uint8_t Bytes)
{
  uint8_t tx[255] = {0};
  uint8_t rx[255] = {0};

  tx[0] = ReadAddr | READ_FLAG;

  spidev_->Transfer(tx, rx, Bytes + 1);

  for (uint8_t i = 0; i < Bytes; i++)
    ReadBuf[i] = rx[i + 1];

  usleep(50);
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and MPU9250 are working correctly.
returns true if mpu9250 answers
-----------------------------------------------------------------------------------------------*/

bool MPU9250::probe()
{
  uint8_t responseXG = ReadReg(MPUREG_WHOAMI | READ_FLAG);

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
  uint8_t responseM = ReadReg(MPUREG_EXT_SENS_DATA_00);

  return (responseXG == 0x71 && responseM == 0x48);
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

#define MPU_InitRegNum 16

void MPU9250::initialize()
{
  uint8_t i = 0;
  uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
      //{0x80, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it
      //seems to corrupt initialisation of AK8963
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
      {0x01, MPUREG_I2C_SLV0_DO},    // Reset AK8963
      {0x81, MPUREG_I2C_SLV0_CTRL},  // Enable I2C and set 1 byte

      {AK8963_CNTL1, MPUREG_I2C_SLV0_REG},  // I2C slave 0 register address from
                                            // where to begin data transfer
      {0x12, MPUREG_I2C_SLV0_DO},   // Register value to continuous measurement
                                    // in 16bit
      {0x81, MPUREG_I2C_SLV0_CTRL}  // Enable I2C and set 1 byte

  };

  SetAccelerometerScale(BITS_FS_16G);
  SetGyroScale(BITS_FS_2000DPS);

  for (i = 0; i < MPU_InitRegNum; i++)
  {
    WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
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

uint8_t MPU9250::SetAccelerometerScale(uint8_t scale)
{
  uint8_t temp_scale;
  WriteReg(MPUREG_ACCEL_CONFIG, scale);

  switch (scale)
  {
    case BITS_FS_2G:
      acc_divider = 16384;
      break;
    case BITS_FS_4G:
      acc_divider = 8192;
      break;
    case BITS_FS_8G:
      acc_divider = 4096;
      break;
    case BITS_FS_16G:
      acc_divider = 2048;
      break;
  }
  temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);

  switch (temp_scale)
  {
    case BITS_FS_2G:
      temp_scale = 2;
      break;
    case BITS_FS_4G:
      temp_scale = 4;
      break;
    case BITS_FS_8G:
      temp_scale = 8;
      break;
    case BITS_FS_16G:
      temp_scale = 16;
      break;
  }
  return temp_scale;
}

/*-----------------------------------------------------------------------------------------------
                                GYROSCOPE SCALE
usage: call this function at startup, after initialization, to set the right
range for the gyroscopes. Suitable ranges are: BITS_FS_250DPS BITS_FS_500DPS
BITS_FS_1000DPS
BITS_FS_2000DPS
returns the range set (250,500,1000 or 2000)
-----------------------------------------------------------------------------------------------*/

uint8_t MPU9250::SetGyroScale(uint8_t scale)
{
  uint8_t temp_scale;
  WriteReg(MPUREG_GYRO_CONFIG, scale);
  switch (scale)
  {
    case BITS_FS_250DPS:
      gyro_divider = 131;
      break;
    case BITS_FS_500DPS:
      gyro_divider = 65.5;
      break;
    case BITS_FS_1000DPS:
      gyro_divider = 32.8;
      break;
    case BITS_FS_2000DPS:
      gyro_divider = 16.4;
      break;
  }

  temp_scale = WriteReg(MPUREG_GYRO_CONFIG | READ_FLAG, 0x00);
  switch (temp_scale)
  {
    case BITS_FS_250DPS:
      temp_scale = 250;
      break;
    case BITS_FS_500DPS:
      temp_scale = 500;
      break;
    case BITS_FS_1000DPS:
      temp_scale = 1000;
      break;
    case BITS_FS_2000DPS:
      temp_scale = 2000;
      break;
  }
  return temp_scale;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected
axis: 0 -> X axis 1 -> Y axis 2 -> Z axis returns Factory Trim value
-----------------------------------------------------------------------------------------------*/

void MPU9250::CalibAccelerometer()
{
  uint8_t response[4];
  int temp_scale;
  // read current acc scale
  temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
  SetAccelerometerScale(BITS_FS_8G);
  // ENABLE SELF TEST need modify
  // temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

  ReadRegs(MPUREG_SELF_TEST_X, response, 4);
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
  uint8_t response[3];
  float data;
  int i;

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
  ReadRegs(MPUREG_EXT_SENS_DATA_00, response, 3);

  // response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
  for (i = 0; i < 3; i++)
  {
    data = response[i];
    magnetometer_asa[i] =
        ((data - 128) / 256 + 1) * Magnetometer_Sensitivity_Scale_Factor;
  }
}

//-----------------------------------------------------------------------------------------------

void MPU9250::update()
{
  uint8_t response[21];
  int16_t bit_data[3];
  int i;
  core::utils::ImuData data;
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

  ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);

  // Get accelerometer value
  for (i = 0; i < 3; i++)
  {
    bit_data[i] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  }
  data.accel[0] = G_SI * bit_data[0] / acc_divider;
  data.accel[1] = G_SI * bit_data[1] / acc_divider;
  data.accel[2] = G_SI * bit_data[2] / acc_divider;

  // Get temperature
  bit_data[0] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  data.temp = ((bit_data[0] - 21) / 333.87) + 21;

  // Get gyroscope value
  for (i = 4; i < 7; i++)
  {
    bit_data[i - 4] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  }
  data.gyro[0] = (PI / 180) * bit_data[0] / gyro_divider;
  data.gyro[1] = (PI / 180) * bit_data[1] / gyro_divider;
  data.gyro[2] = (PI / 180) * bit_data[2] / gyro_divider;

  // Get Magnetometer value
  for (i = 7; i < 10; i++)
  {
    bit_data[i - 7] = ((int16_t)response[i * 2 + 1] << 8) | response[i * 2];
  }
  data.mag[0] = bit_data[0] * magnetometer_asa[0];
  data.mag[1] = bit_data[1] * magnetometer_asa[1];
  data.mag[2] = bit_data[2] * magnetometer_asa[2];
  SetData(data);
}
