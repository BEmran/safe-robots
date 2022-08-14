#pragma once

#include "MPU.h"

class MPU6xx0 : public MPUIMU {

    public:

        Error_t begin(void);

        bool        checkNewData(void);

        void        lowPowerAccelOnly(void);

        void        readGyrometer(float & gx, float & gy, float & gz);

        float       readTemperature(void);

    protected:

        MPU6xx0(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor);

        bool     selfTest(void);

    private:

        void     init(void);
}; 
