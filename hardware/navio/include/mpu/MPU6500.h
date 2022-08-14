#pragma once

#include "MPU6x00.h"

class MPU6500 : public MPU6x00 {

    friend class MPU9250; // eventually should probably subclass MPU9250 from MPU6500

    public:

        MPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor=0);

        Error_t begin(void);

        bool checkNewData(void);

        void readGyrometer(float & gx, float & gy, float & gz);

    protected:

        // Register map
        static const uint8_t SELF_TEST_X_GYRO  = 0x00;                  
        static const uint8_t SELF_TEST_Y_GYRO  = 0x01;
        static const uint8_t SELF_TEST_Z_GYRO  = 0x02;

        // virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;
}; 
