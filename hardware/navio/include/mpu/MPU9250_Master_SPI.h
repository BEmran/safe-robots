#pragma once

#include "MPU9250_Master.h"

class MPU9250_Master_SPI : public MPU9250_Master {

    public:

        MPU9250_Master_SPI(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor=0);

        Error_t begin(void);

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

        virtual void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * data) override;

        virtual void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) override;
};
