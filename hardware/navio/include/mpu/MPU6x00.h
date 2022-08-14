
#pragma once

#include "MPU6xx0.h"

class MPU6x00 : public MPU6xx0 {

    public:

        MPU6x00(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor=0);

    protected:

        Error_t begin(void);
}; 
