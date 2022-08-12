/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.
Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).
To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag -i [sensor name]
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
For print help:
./AccelGyroMag -h
*/

#include "navio/old_mpu9250.h"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

//=============================================================================
int main(int /*argc*/, char ** /*argv[]*/)
{
    if (CheckApm()) {
        return 1;
    }

    auto sensor = std::make_unique<OLD_MPU9250>();

    if (!sensor) {
        printf("Wrong sensor name. Select: mpu or lsm\n");
        return EXIT_FAILURE;
    }

    if (!sensor->Probe()) {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }
    sensor->Initialize();
//-------------------------------------------------------------------------

    while(1) {
        sensor->Update();
        auto data = sensor->GetData();
        printf("Acc: %+7.3f %+7.3f %+7.3f  ", data.accel[0], data.accel[1], data.accel[2]);
        printf("Gyr: %+8.3f %+8.3f %+8.3f  ", data.gyro[0], data.gyro[1], data.gyro[2]);
        printf("Mag: %+7.3f %+7.3f %+7.3f\n", data.mag[0], data.mag[1], data.mag[2]);

       usleep(500000);
    }
}