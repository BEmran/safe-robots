#include "mpu/MPU6x00.h"

MPU6x00::MPU6x00(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor) 
    : MPU6xx0(ascale, gscale, sampleRateDivisor)
{
}

MPUIMU::Error_t MPU6x00::begin(void)
{
    //if (getId() != MPU_ADDRESS) {
    //    return ERROR_IMU_ID;
    //}

    //if (!selfTest()) {
    //    return ERROR_SELFTEST;
    //}

    writeMPURegister(PWR_MGMT_1, 0x80);
    delay(100);

    writeMPURegister(SIGNAL_PATH_RESET, 0x80);
    delay(100);

    writeMPURegister(PWR_MGMT_1, 0x00);
    delay(100);

    writeMPURegister(PWR_MGMT_1, INV_CLK_PLL);
    delay(15);

    writeMPURegister(GYRO_CONFIG, _gScale << 3);
    delay(15);

    writeMPURegister(ACCEL_CONFIG, _aScale << 3);
    delay(15);

    writeMPURegister(CONFIG, 0); // no DLPF bits
    delay(15);

    writeMPURegister(SMPLRT_DIV, _sampleRateDivisor); 
    delay(100);

    // Data ready interrupt configuration
    writeMPURegister(INT_PIN_CFG, 0x10);  
    delay(15);

    writeMPURegister(INT_ENABLE, 0x01); 
    delay(15);

    _accelBias[0] = 0;
    _accelBias[1] = 0;
    _accelBias[2] = 0;

    return ERROR_NONE;
}