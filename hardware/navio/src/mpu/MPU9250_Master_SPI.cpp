#include "mpu/MPU9250_Master_SPI.h"
#include "navio/old_spidev.h"

MPU9250_Master_SPI::MPU9250_Master_SPI(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor) :
    MPU9250_Master(ascale, gscale, mscale, mmode, sampleRateDivisor)
{
}

MPUIMU::Error_t MPU9250_Master_SPI::begin(void)
{
    return runTests();
}

void MPU9250_Master_SPI::readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data)
{
    (void)address;
    readMPURegisters(subAddress, count, data);
}


void MPU9250_Master_SPI::writeRegister(uint8_t address, uint8_t subAddress, uint8_t data)
{
    (void)address;
    writeMPURegister(subAddress, data);
}

void MPU9250_Master_SPI::writeMPURegister(uint8_t subAddress, uint8_t data)
{
    uint8_t buf[2] = {subAddress, data};
    OLDSPIdev::transfer("/dev/spidev0.1", buf, 2);
}

void MPU9250_Master_SPI::readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    unsigned char buf[count + 1] = {0};
    buf[0] = subAddress | 0x80;
    OLDSPIdev::transfer("/dev/spidev0.1", buf, count + 1);

    for(uint8_t i=0; i < count; i++)
        dest[i] = buf[i + 1];

    usleep(50);
}
