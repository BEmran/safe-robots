#include "core/utils/data.hpp"

namespace core::utils {
std::ostream& operator<<(std::ostream& os, const BarData& bar) {
  return os << std::setprecision(HALF_PRECISION)
            << "Barometer data: " << bar.value;
}

std::ostream& operator<<(std::ostream& os, const TemperatureData& temp) {
  return os << std::setprecision(HALF_PRECISION) << "Temp (C): " << temp.value;
}

std::ostream& operator<<(std::ostream& os, const HeadingData& head) {
  return os << std::setprecision(HALF_PRECISION)
            << "heading (rad): " << head.value;
}

std::ostream& operator<<(std::ostream& os, const AdcData& adc) {
  return os << "ADC data: " << adc.values.transpose();
}

std::ostream& operator<<(std::ostream& os, const GpsData& gps) {
  return os << "GPS data: " << std::setprecision(FULL_PRECISION) /* precision */
            << "\n- Lat: " << gps.lat                            /* latitude */
            << "\n- Lon: " << gps.lon                            /* longitude */
            << "\n- Alt: " << gps.alt << std::endl;              /* altitude */
}

std::ostream& operator<<(std::ostream& os, const Vec3Data& vec) {
  return os << vec.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const AccelData& accel) {
  return os << "Accel XYZ(m/s^2): " << accel.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const GyroData& gyro) {
  return os << "Gyro  XYZ(rad/s): " << gyro.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const MagData& mag) {
  return os << "Mag Field XYZ(uT): " << mag.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const RPYData& rpy) {
  return os << "TaitBryan RPY(rad): " << rpy.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const QuatData& quat) {
  return os << "Quat WXYZ: " << quat.data;
}

std::ostream& operator<<(std::ostream& os, const ImuData& imu) {
  return os << "IMU data:"
            << "\n- " << imu.accel      /* acceleromter data */
            << "\n- " << imu.gyro       /* gyroscopic data */
            << "\n- " << imu.mag        /* magnetometer data */
            << "\n- " << imu.quat       /* quaternion data */
            << "\n- " << imu.tait_bryan /* tait bryan data */
            << "\n- " << imu.heading    /* heading angle data */
            << "\n- " << imu.temp       /* temperature bryan data */
            << std::endl;
}
}  // namespace core::utils
