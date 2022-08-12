
#include "navio/mpu9250.hpp"
#include <core/utils/math.hpp>

constexpr auto G_SI = core::utils::GRAVITY;
constexpr auto PI = core::utils::PI;
namespace
{
/* MPU9250 registers */
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
constexpr uint8_t MPUREGSiGNAL_PATH_RESET = 0x68;
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

/* AK8963 Reg In MPU9250 */
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

// Read-only Reg (ROM)
constexpr uint8_t AK8963_ASAX = 0x10;
constexpr uint8_t AK8963_ASAY = 0x11;
constexpr uint8_t AK8963_ASAZ = 0x12;

// Configuration bits MPU9250
constexpr uint8_t BIT_SLEEP = 0x40;
constexpr uint8_t BIT_H_RESET = 0x80;
constexpr uint8_t BITS_CLKSEL = 0x07;
constexpr uint8_t MPU_CLK_SEL_PLLGYROX = 0x01;
constexpr uint8_t MPU_CLK_SEL_PLLGYROZ = 0x03;
constexpr uint8_t MPU_EXT_SYNC_GYROX = 0x02;
constexpr uint8_t BITS_FS_250DPS = 0x00;
constexpr uint8_t BITS_FS_500DPS = 0x08;
constexpr uint8_t BITS_FS_1000DPS = 0x10;
constexpr uint8_t BITS_FS_2000DPS = 0x18;
constexpr uint8_t BITS_FS_2G = 0x00;
constexpr uint8_t BITS_FS_4G = 0x08;
constexpr uint8_t BITS_FS_8G = 0x10;
constexpr uint8_t BITS_FS_16G = 0x18;
constexpr uint8_t BITS_FS_MASK = 0x18;
constexpr uint8_t BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00;
constexpr uint8_t BITS_DLPF_CFG_188HZ = 0x01;
constexpr uint8_t BITS_DLPF_CFG_98HZ = 0x02;
constexpr uint8_t BITS_DLPF_CFG_42HZ = 0x03;
constexpr uint8_t BITS_DLPF_CFG_20HZ = 0x04;
constexpr uint8_t BITS_DLPF_CFG_10HZ = 0x05;
constexpr uint8_t BITS_DLPF_CFG_5HZ = 0x06;
constexpr uint8_t BITS_DLPF_CFG_2100HZ_NOLPF = 0x07;
constexpr uint8_t BITS_DLPF_CFG_MASK = 0x07;
constexpr uint8_t BIT_INT_ANYRD_2CLEAR = 0x10;
constexpr uint8_t BIT_RAW_RDY_EN = 0x01;
constexpr uint8_t BIT_I2C_IF_DIS = 0x10;

constexpr uint8_t READ_FLAG = 0x80;

/* Sensitivity */
constexpr double MPU9250A_2g = 0.000061035156;       // 0.000061035156 g/LSB
constexpr double MPU9250A_4g = 0.000122070312;       // 0.000122070312 g/LSB
constexpr double MPU9250A_8g = 0.000244140625;       // 0.000244140625 g/LSB
constexpr double MPU9250A_16g = 0.000488281250;      // 0.000488281250 g/LSB
constexpr double MPU9250G_250dps = 0.007633587786;   // 0.007633587786 dps/LSB
constexpr double MPU9250G_500dps = 0.015267175572;   // 0.015267175572 dps/LSB
constexpr double MPU9250G_1000dps = 0.030487804878;  // 0.030487804878 dps/LSB
constexpr double MPU9250G_2000dps = 0.060975609756;  // 0.060975609756 dps/LSB
constexpr double MPU9250T_85degC = 0.002995177763;   // 0.002995177763 degC/LSB
constexpr double MPU9250M_4800uT = 0.6;              // 0.6 uT/LSB
constexpr double Magnetometer_Sensitivity_Scale_Factor = 0.15;
}  // namespace

/*-----------------------------------------------------------------------------------------------
REGISTER READ & WRITE
usage: use these methods to read and write MPU9250 registers over SPI
-----------------------------------------------------------------------------------------------*/

uint8_t MPU9250::WriteReg(uint8_t WriteAddr, uint8_t WriteData)
{
  uint8_t tx[2] = {WriteAddr, WriteData};
  uint8_t rx[2] = {0, 0};

  // SPIdev::transfer("/dev/spidev0.1", tx, rx, 2);

  return rx[1];
}

uint8_t MPU9250::ReadReg(uint8_t ReadAddr)
{
  return WriteReg(ReadAddr | READ_FLAG, 0x00);
}

void MPU9250::ReadRegs(uint8_t ReadAddr, uint8_t* ReadBuf, uint8_t Bytes)
{
  unsigned int i = 0;

  unsigned char tx[255] = {0};
  unsigned char rx[255] = {0};

  tx[0] = ReadAddr | READ_FLAG;

  // SPIdev::transfer("/dev/spidev0.1", tx, rx, Bytes + 1);

  for (i = 0; i < Bytes; i++)
  {
    ReadBuf[i] = rx[i + 1];
  }
  // usleep(50);
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
           0x0D);  // I2C configuration multi-master IIC 400KHz
  WriteReg(MPUREG_I2C_SLV0_ADDR,
           AK8963_I2C_ADDR | READ_FLAG);  // Set the I2C slave addres of AK8963
                                          // and set for read.
  WriteReg(MPUREG_I2C_SLV0_REG,
           AK8963_WIA);  // I2C slave 0 register address from where to begin
                         // data transfer
  WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);  // Read 1 byte from the magnetometer
  // usleep(10000);
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

bool MPU9250::initialize()
{
  constexpr uint8_t MPU_InitRegNum = 16;
  uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
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

  SetAcceleometerScale(BITS_FS_16G);
  SetGyroCalibrationScale(BITS_FS_2000DPS);

  for (uint8_t i = 0; i < MPU_InitRegNum; i++)
  {
    WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
    // usleep(100000);  // I2C must slow down the write speed, otherwise it won't
                        // work
  }

  CalibrateMagnetometer();
  return 0;
}
/*-----------------------------------------------------------------------------------------------
ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right
range for the accelerometers. Suitable ranges are: BITS_FS_2G BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/

unsigned int MPU9250::SetAcceleometerScale(uint8_t scale)
{
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

  uint8_t temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
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

unsigned int MPU9250::SetGyroCalibrationScale(uint8_t scale)
{
  unsigned int temp_scale;
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

void MPU9250::CalibrateAcceleometer()
{
  uint8_t response[4];
  int temp_scale;
  // read current acc scale
  temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | READ_FLAG, 0x00);
  SetAcceleometerScale(BITS_FS_8G);
  // ENABLE SELF TEST need modify
  // temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

  ReadRegs(MPUREG_SELF_TEST_X, response, 4);
  calib_data[0] =
      ((response[0] & 11100000) >> 3) | ((response[3] & 00110000) >> 4);
  calib_data[1] =
      ((response[1] & 11100000) >> 3) | ((response[3] & 00001100) >> 2);
  calib_data[2] = ((response[2] & 11100000) >> 3) | ((response[3] & 00000011));

  SetAcceleometerScale(temp_scale);
}

//-----------------------------------------------------------------------------------------------

void MPU9250::CalibrateMagnetometer()
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
  uint8_t response[3];
  ReadRegs(MPUREG_EXT_SENS_DATA_00, response, 3);

  // response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
  for (int i = 0; i < 3; i++)
  {
    float data = response[i];
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
  data_.accel[0] = G_SI * bit_data[0] / acc_divider;
  data_.accel[1] = G_SI * bit_data[1] / acc_divider;
  data_.accel[2] = G_SI * bit_data[2] / acc_divider;

  // Get temperature
  bit_data[0] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  data_.temp = ((bit_data[0] - 21) / 333.87) + 21;

  // Get gyroscope value
  for (i = 4; i < 7; i++)
  {
    bit_data[i - 4] = ((int16_t)response[i * 2] << 8) | response[i * 2 + 1];
  }
  data_.gyro[0] = (PI / 180) * bit_data[0] / gyro_divider;
  data_.gyro[1] = (PI / 180) * bit_data[1] / gyro_divider;
  data_.gyro[2] = (PI / 180) * bit_data[2] / gyro_divider;

  // Get Magnetometer value
  for (i = 7; i < 10; i++)
  {
    bit_data[i - 7] = ((int16_t)response[i * 2 + 1] << 8) | response[i * 2];
  }
  data_.mag[0] = bit_data[0] * magnetometer_asa[0];
  data_.mag[1] = bit_data[1] * magnetometer_asa[1];
  data_.mag[2] = bit_data[2] * magnetometer_asa[2];

  SetData();
}