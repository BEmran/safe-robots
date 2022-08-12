#include "core/utils/data.hpp"

namespace core::utils
{

std::ostream& operator<<(std::ostream& os, const AdcData& adc)
{
  return os << "ADC data: " << adc.values.transpose() << std::endl;
}

std::ostream& operator<<(std::ostream& os, const BarData& bar)
{
  return os << "Barometer data: " << bar.value << std::endl;
}

std::ostream& operator<<(std::ostream& os, const GpsData& gps)
{
  return os << "GPS data: " << std::setprecision(COUT_PRECISION) /* precision */
            << "\n- Lat: " << gps.lat                        /* latitude */
            << "\n- Lon: " << gps.lon                        /* longitude */
            << "\n- Alt: " << gps.alt << std::endl;          /* altitude */
}

std::ostream& operator<<(std::ostream& os, const ImuData& imu)
{
  return os << "IMU data:"
            << "\n- Accel XYZ(m/s^2): " << imu.accel.transpose()
            << "\n- Gyro  XYZ(rad/s): " << imu.gyro.transpose()
            << "\n- Mag Field XYZ(uT): " << imu.mag.transpose()
            << "\n- Quat WXYZ: " << imu.quat
            << "\n- TaitBryan RPY(rad): " << imu.tait_bryan.transpose()
            << std::setprecision(COUT_PRECISION) /* set precision */
            << "\n- heading (rad): " << imu.heading
            << "\n- Temp (C): " << imu.temp << std::endl;
}
}  // namespace core::utils