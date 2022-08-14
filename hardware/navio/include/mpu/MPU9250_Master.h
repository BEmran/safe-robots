#pragma once

#include "MPU9250.h"

class MPU9250_Master : public MPU9250 {

    public:

        MPU9250_Master(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor=0);

        bool checkNewData(void);

    protected:

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) override;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) override;


    private:

        void initMPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor);
};
