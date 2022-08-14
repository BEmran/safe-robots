#include "mpu/MPU6500.h"

MPU6500::MPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor) 
    : MPU6x00(ascale, gscale, sampleRateDivisor)
{
}

MPUIMU::Error_t MPU6500::begin(void)
{
    return MPU6x00::begin();
}

bool MPU6500::checkNewData(void)
{
    return MPUIMU::checkNewData();
}

void MPU6500::readGyrometer(float & gx, float & gy, float & gz)
{
    MPUIMU::readGyrometer(gx, gy, gz);
}

// void MPU6500::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) 
// {
    // cpspi_readRegisters(subAddress | 0x80, count, dest);
// }
