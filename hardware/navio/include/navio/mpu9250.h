/*
Written by Qiyong Mu (kylongmu@msn.com)
Adapted for Raspberry Pi by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
*/

#ifndef _MPU9250_H
#define _MPU9250_H

#include "spidev.h"
#include <core/sensors/module_sensor_imu.hpp>
#include <memory>

class MPU9250 : public core::sensors::ImuSensorModule
{
public:
    MPU9250(const bool debug);

    void initialize() override;
    bool probe() override;
    void update() override;

private:
    uint8_t WriteReg(uint8_t WriteAddr, uint8_t WriteData);
    uint8_t ReadReg(uint8_t ReadAddr);
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, uint8_t Bytes);

    uint8_t SetGyroScale(uint8_t scale);
    uint8_t SetAccelerometerScale(uint8_t scale);

    void CalibAccelerometer();
    void CalibMagnetometer();

    float acc_divider;
    float gyro_divider;

    int calib_data[3];
    float magnetometer_asa[3];
    std::unique_ptr<SPIdev> spidev_;
};

#endif //_MPU9250_H

// Configuration bits MPU9250
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10

#define READ_FLAG                   0x80

/* ---- Sensitivity --------------------------------------------------------- */

#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   ((float)0.002995177763f) // 0.002995177763 degC/LSB

#define Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)
