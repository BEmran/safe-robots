#ifndef MPU_SENSOR_HPP
#define MPU_SENSOR_HPP

#include <unistd.h>
#include <stdint.h>

void delay(uint32_t msec);

enum class Error_t
{
  ERROR_NONE,
  ERROR_CONNECT,
  ERROR_IMU_ID,
  ERROR_MAG_ID,
  ERROR_SELFTEST
};
template <typedef T>
class Sensor
{
 public:
 T Read();
 private:
 T data;
};

#endif  // MPU_SENSOR_HPP