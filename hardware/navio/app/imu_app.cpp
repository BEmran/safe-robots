/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com
Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.
Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro
LSM9DS1) and /dev/spidev0.2 (mag LSM9DS1). To run this example navigate to the
directory containing it and run following commands: make
./AccelGyroMag -i .data[sensor name]
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
For print help:
./AccelGyroMag -h
*/

#include "navio/mpu9250.h"
#include "navio/old_mpu9250.h"
#include "navio/util.h"
#include <unistd.h>
#include <string>
#include <memory>

std::unique_ptr<core::sensors::ImuSensorModule>
GetInertialSensor(std::string sensor_name)
{
  if (sensor_name == "new")
  {
    printf("Selected: MPU9250\n");
    auto ptr = std::make_unique<MyMPU9250>(false);
    return ptr;
  }
  else if (sensor_name == "old")
  {
    printf("Selected: MPU9250\n");
    auto ptr = std::make_unique<OLD_MPU9250>();
    return ptr;
  }
  else
  {
    return NULL;
  }
}

void PrintHelp()
{
  printf("Possible parameters:\nSensor selection: -i .data[sensor name]\n");
  printf("Sensors names: new is MPU9250, old is LSM9DS1\nFor help: -h\n");
}

std::string GetSensorName(int argc, char* argv[])
{
  if (GetNavioVersion() == NAVIO2)
  {
    if (argc < 2)
    {
      printf("Enter parameter\n");
      PrintHelp();
      return std::string();
    }

    // prevent the error message
    opterr = 0;
    int parameter;

    while ((parameter = getopt(argc, argv, "i:h")) != -1)
    {
      switch (parameter)
      {
        case 'i':
          return optarg;
        case 'h':
          PrintHelp();
          return "-1";
        case '?':
          printf("Wrong parameter.\n");
          PrintHelp();
          return std::string();
      }
    }
  }
  else
  {  // sensor on NAVIO+

    return "new";
  }

  return std::string();
}
//=============================================================================
int main(int argc, char* argv[])
{
  if (CheckApm())
  {
    return 1;
  }

  auto sensor_name = GetSensorName(argc, argv);
  if (sensor_name.empty())
  {
    return EXIT_FAILURE;
  }
  // auto sensor_name = "mpu";
  auto sensor = GetInertialSensor(sensor_name);

  if (!sensor)
  {
    printf("Wrong sensor name. Select: new or old\n");
    return EXIT_FAILURE;
  }

  if (!sensor->Probe())
  {
    printf("Sensor not enabled\n");
    return EXIT_FAILURE;
  }
  // return EXIT_FAILURE;
  sensor->Initialize();
  //-------------------------------------------------------------------------

  while (1)
  {
    sensor->Update();
    auto data = sensor->GetData();
    std::cout << data.accel << " \t" << data.gyro << " \t" << data.mag << " \t"
              << data.temp << std::endl;
    usleep(500000);
  }
}