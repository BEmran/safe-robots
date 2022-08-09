#ifndef HARDWARE_NAVIO2_MPU9250_HPP
#define HARDWARE_NAVIO2_MPU9250_HPP

#include "SPIdev.h"
#include "core/sensors/module_sensor_imu.hpp"
using core::sensors::ImuSensorModule;
class MPU9250 : public ImuSensorModule

{
 public:
  MPU9250();

  bool initialize();
  bool probe();
  void update();

 private:
  unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData);
  unsigned int ReadReg(uint8_t ReadAddr);
  void ReadRegs(uint8_t ReadAddr, uint8_t* ReadBuf, unsigned int Bytes);

  unsigned int set_gyro_scale(int scale);
  unsigned int set_acc_scale(int scale);

  void calib_acc();
  void calib_mag();

  float acc_divider = 1.0;
  float gyro_divider = 1.0;

  int calib_data[3] = {0, 0, 0};
  float magnetometer_ASA[3] = {1.0, 1.0, 1.0} ;
};

// MPU9250 registers
constexpr auto MPUREG_XG_OFFS_TC = 0x00;
constexpr auto MPUREG_YG_OFFS_TC = 0x01;
constexpr auto MPUREG_ZG_OFFS_TC = 0x02;
constexpr auto MPUREG_X_FINE_GAIN = 0x03;
constexpr auto MPUREG_Y_FINE_GAIN = 0x04;
constexpr auto MPUREG_Z_FINE_GAIN = 0x05;
constexpr auto MPUREG_XA_OFFS_H = 0x06;
constexpr auto MPUREG_XA_OFFS_L = 0x07;
constexpr auto MPUREG_YA_OFFS_H = 0x08;
constexpr auto MPUREG_YA_OFFS_L = 0x09;
constexpr auto MPUREG_ZA_OFFS_H = 0x0A;
constexpr auto MPUREG_ZA_OFFS_L = 0x0B;
constexpr auto MPUREG_PRODUCT_ID = 0x0C;
constexpr auto MPUREG_SELF_TEST_X = 0x0D;
constexpr auto MPUREG_SELF_TEST_Y = 0x0E;
constexpr auto MPUREG_SELF_TEST_Z = 0x0F;
constexpr auto MPUREG_SELF_TEST_A = 0x10;
constexpr auto MPUREG_XG_OFFS_USRH = 0x13;
constexpr auto MPUREG_XG_OFFS_USRL = 0x14;
constexpr auto MPUREG_YG_OFFS_USRH = 0x15;
constexpr auto MPUREG_YG_OFFS_USRL = 0x16;
constexpr auto MPUREG_ZG_OFFS_USRH = 0x17;
constexpr auto MPUREG_ZG_OFFS_USRL = 0x18;
constexpr auto MPUREG_SMPLRT_DIV = 0x19;
constexpr auto MPUREG_CONFIG = 0x1A;
constexpr auto MPUREG_GYRO_CONFIG = 0x1B;
constexpr auto MPUREG_ACCEL_CONFIG = 0x1C;
constexpr auto MPUREG_ACCEL_CONFIG_2 = 0x1D;
constexpr auto MPUREG_LP_ACCEL_ODR = 0x1E;
constexpr auto MPUREG_MOT_THR = 0x1F;
constexpr auto MPUREG_FIFO_EN = 0x23;
constexpr auto MPUREG_I2C_MST_CTRL = 0x24;
constexpr auto MPUREG_I2C_SLV0_ADDR = 0x25;
constexpr auto MPUREG_I2C_SLV0_REG = 0x26;
constexpr auto MPUREG_I2C_SLV0_CTRL = 0x27;
constexpr auto MPUREG_I2C_SLV1_ADDR = 0x28;
constexpr auto MPUREG_I2C_SLV1_REG = 0x29;
constexpr auto MPUREG_I2C_SLV1_CTRL = 0x2A;
constexpr auto MPUREG_I2C_SLV2_ADDR = 0x2B;
constexpr auto MPUREG_I2C_SLV2_REG = 0x2C;
constexpr auto MPUREG_I2C_SLV2_CTRL = 0x2D;
constexpr auto MPUREG_I2C_SLV3_ADDR = 0x2E;
constexpr auto MPUREG_I2C_SLV3_REG = 0x2F;
constexpr auto MPUREG_I2C_SLV3_CTRL = 0x30;
constexpr auto MPUREG_I2C_SLV4_ADDR = 0x31;
constexpr auto MPUREG_I2C_SLV4_REG = 0x32;
constexpr auto MPUREG_I2C_SLV4_DO = 0x33;
constexpr auto MPUREG_I2C_SLV4_CTRL = 0x34;
constexpr auto MPUREG_I2C_SLV4_DI = 0x35;
constexpr auto MPUREG_I2C_MST_STATUS = 0x36;
constexpr auto MPUREG_INT_PIN_CFG = 0x37;
constexpr auto MPUREG_INT_ENABLE = 0x38;
constexpr auto MPUREG_ACCEL_XOUT_H = 0x3B;
constexpr auto MPUREG_ACCEL_XOUT_L = 0x3C;
constexpr auto MPUREG_ACCEL_YOUT_H = 0x3D;
constexpr auto MPUREG_ACCEL_YOUT_L = 0x3E;
constexpr auto MPUREG_ACCEL_ZOUT_H = 0x3F;
constexpr auto MPUREG_ACCEL_ZOUT_L = 0x40;
constexpr auto MPUREG_TEMP_OUT_H = 0x41;
constexpr auto MPUREG_TEMP_OUT_L = 0x42;
constexpr auto MPUREG_GYRO_XOUT_H = 0x43;
constexpr auto MPUREG_GYRO_XOUT_L = 0x44;
constexpr auto MPUREG_GYRO_YOUT_H = 0x45;
constexpr auto MPUREG_GYRO_YOUT_L = 0x46;
constexpr auto MPUREG_GYRO_ZOUT_H = 0x47;
constexpr auto MPUREG_GYRO_ZOUT_L = 0x48;
constexpr auto MPUREG_EXT_SENS_DATA_00 = 0x49;
constexpr auto MPUREG_EXT_SENS_DATA_01 = 0x4A;
constexpr auto MPUREG_EXT_SENS_DATA_02 = 0x4B;
constexpr auto MPUREG_EXT_SENS_DATA_03 = 0x4C;
constexpr auto MPUREG_EXT_SENS_DATA_04 = 0x4D;
constexpr auto MPUREG_EXT_SENS_DATA_05 = 0x4E;
constexpr auto MPUREG_EXT_SENS_DATA_06 = 0x4F;
constexpr auto MPUREG_EXT_SENS_DATA_07 = 0x50;
constexpr auto MPUREG_EXT_SENS_DATA_08 = 0x51;
constexpr auto MPUREG_EXT_SENS_DATA_09 = 0x52;
constexpr auto MPUREG_EXT_SENS_DATA_10 = 0x53;
constexpr auto MPUREG_EXT_SENS_DATA_11 = 0x54;
constexpr auto MPUREG_EXT_SENS_DATA_12 = 0x55;
constexpr auto MPUREG_EXT_SENS_DATA_13 = 0x56;
constexpr auto MPUREG_EXT_SENS_DATA_14 = 0x57;
constexpr auto MPUREG_EXT_SENS_DATA_15 = 0x58;
constexpr auto MPUREG_EXT_SENS_DATA_16 = 0x59;
constexpr auto MPUREG_EXT_SENS_DATA_17 = 0x5A;
constexpr auto MPUREG_EXT_SENS_DATA_18 = 0x5B;
constexpr auto MPUREG_EXT_SENS_DATA_19 = 0x5C;
constexpr auto MPUREG_EXT_SENS_DATA_20 = 0x5D;
constexpr auto MPUREG_EXT_SENS_DATA_21 = 0x5E;
constexpr auto MPUREG_EXT_SENS_DATA_22 = 0x5F;
constexpr auto MPUREG_EXT_SENS_DATA_23 = 0x60;
constexpr auto MPUREG_I2C_SLV0_DO = 0x63;
constexpr auto MPUREG_I2C_SLV1_DO = 0x64;
constexpr auto MPUREG_I2C_SLV2_DO = 0x65;
constexpr auto MPUREG_I2C_SLV3_DO = 0x66;
constexpr auto MPUREG_I2C_MST_DELAY_CTRL = 0x67;
constexpr auto MPUREG_SIGNAL_PATH_RESET = 0x68;
constexpr auto MPUREG_MOT_DETECT_CTRL = 0x69;
constexpr auto MPUREG_USER_CTRL = 0x6A;
constexpr auto MPUREG_PWR_MGMT_1 = 0x6B;
constexpr auto MPUREG_PWR_MGMT_2 = 0x6C;
constexpr auto MPUREG_BANK_SEL = 0x6D;
constexpr auto MPUREG_MEM_START_ADDR = 0x6E;
constexpr auto MPUREG_MEM_R_W = 0x6F;
constexpr auto MPUREG_DMP_CFG_1 = 0x70;
constexpr auto MPUREG_DMP_CFG_2 = 0x71;
constexpr auto MPUREG_FIFO_COUNTH = 0x72;
constexpr auto MPUREG_FIFO_COUNTL = 0x73;
constexpr auto MPUREG_FIFO_R_W = 0x74;
constexpr auto MPUREG_WHOAMI = 0x75;
constexpr auto MPUREG_XA_OFFSET_H = 0x77;
constexpr auto MPUREG_XA_OFFSET_L = 0x78;
constexpr auto MPUREG_YA_OFFSET_H = 0x7A;
constexpr auto MPUREG_YA_OFFSET_L = 0x7B;
constexpr auto MPUREG_ZA_OFFSET_H = 0x7D;
constexpr auto MPUREG_ZA_OFFSET_L = 0x7E;

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
constexpr auto AK8963_I2C_ADDR 0x0c;  // should return 0x18
constexpr auto AK8963_Device_ID 0x48;

// Read-only Reg
constexpr auto AK8963_WIA = 0x00;
constexpr auto AK8963_INFO = 0x01;
constexpr auto AK8963_ST1 = 0x02;
constexpr auto AK8963_HXL = 0x03;
constexpr auto AK8963_HXH = 0x04;
constexpr auto AK8963_HYL = 0x05;
constexpr auto AK8963_HYH = 0x06;
constexpr auto AK8963_HZL = 0x07;
constexpr auto AK8963_HZH = 0x08;
constexpr auto AK8963_ST2 = 0x09;

// Write/Read Reg
constexpr auto AK8963_CNTL1 = 0x0A;
constexpr auto AK8963_CNTL2 = 0x0B;
constexpr auto AK8963_ASTC = 0x0C;
constexpr auto AK8963_TS1 = 0x0D;
constexpr auto AK8963_TS2 = 0x0E;
constexpr auto AK8963_I2CDIS = 0x0F;

// Read-only Reg ( ROM )
constexpr auto AK8963_ASAX = 0x10;
constexpr auto AK8963_ASAY = 0x11;
constexpr auto AK8963_ASAZ = 0x12;

// Configuration bits MPU9250
constexpr auto BIT_SLEEP = 0x40;
constexpr auto BIT_H_RESET = 0x80;
constexpr auto BITS_CLKSEL = 0x07;
constexpr auto MPU_CLK_SEL_PLLGYROX = 0x01;
constexpr auto MPU_CLK_SEL_PLLGYROZ = 0x03;
constexpr auto MPU_EXT_SYNC_GYROX = 0x02;
constexpr auto BITS_FS_250DPS = 0x00;
constexpr auto BITS_FS_500DPS = 0x08;
constexpr auto BITS_FS_1000DPS = 0x10;
constexpr auto BITS_FS_2000DPS = 0x18;
constexpr auto BITS_FS_2G = 0x00;
constexpr auto BITS_FS_4G = 0x08;
constexpr auto BITS_FS_8G = 0x10;
constexpr auto BITS_FS_16G = 0x18;
constexpr auto BITS_FS_MASK = 0x18;
constexpr auto BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00;
constexpr auto BITS_DLPF_CFG_188HZ = 0x01;
constexpr auto BITS_DLPF_CFG_98HZ = 0x02;
constexpr auto BITS_DLPF_CFG_42HZ = 0x03;
constexpr auto BITS_DLPF_CFG_20HZ = 0x04;
constexpr auto BITS_DLPF_CFG_10HZ = 0x05;
constexpr auto BITS_DLPF_CFG_5HZ = 0x06;
constexpr auto BITS_DLPF_CFG_2100HZ_NOLPF = 0x07;
constexpr auto BITS_DLPF_CFG_MASK = 0x07;
constexpr auto BIT_INT_ANYRD_2CLEAR = 0x10;
constexpr auto BIT_RAW_RDY_EN = 0x01;
constexpr auto BIT_I2C_IF_DIS = 0x10;

constexpr auto READ_FLAG = 0x80;

/* ---- Sensitivity --------------------------------------------------------- */

constexpr auto MPU9250A_2g = ((float)0.000061035156f);   // 0.000061035156 g/LSB
constexpr auto MPU9250A_4g = ((float)0.000122070312f);   // 0.000122070312 g/LSB
constexpr auto MPU9250A_8g = ((float)0.000244140625f);   // 0.000244140625 g/LSB
constexpr auto MPU9250A_16g = ((float)0.000488281250f);  // 0.000488281250 g/LSB
constexpr auto MPU9250G_250dps =
    ((float)0.007633587786f);  // 0.007633587786 dps/LSB
constexpr auto MPU9250G_500dps =
    ((float)0.015267175572f);  // 0.015267175572 dps/LSB
constexpr auto MPU9250G_1000dps =
    ((float)0.030487804878f);  // 0.030487804878 dps/LSB
constexpr auto MPU9250G_2000dps =
    ((float)0.060975609756f);                    // 0.060975609756 dps/LSB
constexpr auto MPU9250M_4800uT = ((float)0.6f);  // 0.6 uT/LSB
constexpr auto MPU9250T_85degC =
    ((float)0.002995177763f);  // 0.002995177763 degC/LSB
constexpr auto Magnetometer_Sensitivity_Scale_Factor = ((float)0.15f);

#endif  // HARDWARE_NAVIO2_MPU9250_HPP