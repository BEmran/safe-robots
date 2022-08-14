#include "mpu/MPU9250_Master.h"
#include <iostream>
MPU9250_Master::MPU9250_Master(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor) 
    : MPU9250(ascale, gscale, mscale, mmode, sampleRateDivisor, false)
{
}

void MPU9250_Master::initMPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor) 
{ 
    MPU9250::initMPU6500(ascale, gscale, sampleRateDivisor, false); 
}

void MPU9250_Master::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    uint8_t count = 1;

    writeMPURegister(I2C_SLV0_ADDR, AK8963_ADDRESS); // set slave 0 to the AK8963 and set for write
    writeMPURegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    writeMPURegister(I2C_SLV0_DO, data); // store the data for write
    writeMPURegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and send 1 byte

    // writeMPURegister(USER_CTRL, I2C_MST_EN);  // I2C Master mode
    // writeMPURegister(I2C_MST_CTRL, 0x0D); // I2C configuration multi-master  IIC 400KHz
    // writeMPURegister(I2C_SLV0_ADDR, 0x0C | 0x80); //Set the I2C slave addres of AK8963 and set for read.
    // writeMPURegister(I2C_SLV0_REG, 0x00); //I2C slave 0 register address from where to begin data transfer
    // writeMPURegister(I2C_SLV0_CTRL, 0x81); //Read 1 byte from
}

void MPU9250_Master::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest)
{
    // writeMPURegister(USER_CTRL, I2C_MST_EN);  // I2C Master mode
    // writeMPURegister(I2C_MST_CTRL, 0x0D); // I2C configuration multi-master  IIC 400KHz
    // printf("I2C_SLV0_ADDR:\n");
    writeMPURegister(I2C_SLV0_ADDR, AK8963_ADDRESS | I2C_READ_FLAG); // set slave 0 to the AK8963 and set for read
    // printf("I2C_SLV0_REG:\n");
    writeMPURegister(I2C_SLV0_REG, subAddress); // set the register to the desired AK8963 sub address
    // printf("I2C_SLV0_CTRL:\n");
    writeMPURegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count); // enable I2C and request the bytes
    delay(10); // takes some time for these registers to fill
    // printf("EXT_SENS_DATA_00:\n");
    readMPURegisters(EXT_SENS_DATA_00, count, dest); // read the bytes off the MPU9250 EXT_SENS_DATA registers
}

bool MPU9250_Master::checkNewData(void)
{
    return (readMPURegister(INT_STATUS) & 0x01);
}
