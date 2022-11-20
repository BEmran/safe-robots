#include <core/utils/logger_macros.hpp>
#include <cstdio>
#include <sensors/mpu/mpu.hpp>

int main() {
  SYS_LOG_DEBUG("This program will generate a new gyro calibration file");
  SYS_LOG_DEBUG("Press any key to continue");
  getchar();
  SYS_LOG_DEBUG("Starting calibration routine");

  // use defaults for now, except also enable magnetometer.
  rc_mpu_config_t conf = rc_mpu_default_config();
  conf.show_warnings = true;

  if (not simple_initialize(conf)) {
    SYS_LOG_ERROR("failed to initialize mpu");
    return -1;
  }

  if (not rc_mpu_calibrate_gyro_routine()) {
    SYS_LOG_WARN("Failed to complete gyro calibration");
    return -1;
  }

  SYS_LOG_DEBUG("gyro calibration file written");
  SYS_LOG_DEBUG("run rc_test_mpu to check performance");

  return 0;
}
