// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/data.hpp"

namespace core::utils {
std::string Header(const std::vector<Data*>& vec) {
  std::string header;
  std::for_each(vec.begin(), vec.end(), [&header](const Data* data) {
    header += data->Header() + ", ";
  });
  return header;
}

std::string Header(const std::vector<std::shared_ptr<Data>>& vec) {
  std::string header;
  std::for_each(vec.begin(), vec.end(),
                [&header](const std::shared_ptr<Data> data) {
                  header += data->Header() + ", ";
                });
  return header;
}

std::string ToString(const std::vector<Data*>& vec) {
  std::string str;
  std::for_each(vec.begin(), vec.end(),
                [&str](const Data* data) { str += data->ToString() + ", "; });
  return str;
}

std::string ToString(const std::vector<std::shared_ptr<Data>>& vec) {
  std::string str;
  std::for_each(vec.begin(), vec.end(),
                [&str](const std::shared_ptr<Data> data) {
                  str += data->ToString() + ", ";
                });
  return str;
}
}  // namespace core::utils

namespace cu = core::utils;

std::ostream& operator<<(std::ostream& os, const cu::Data* const data) {
  return os << data->ToString();
}

std::ostream& operator<<(std::ostream& os, const cu::BarData& bar) {
  return os << std::setprecision(cu::HALF_PRECISION)
            << "Barometer data: " << bar.value;
}

std::ostream& operator<<(std::ostream& os, const cu::TemperatureData& temp) {
  return os << std::setprecision(cu::HALF_PRECISION)
            << "Temp (C): " << temp.value;
}

std::ostream& operator<<(std::ostream& os, const cu::HeadingData& head) {
  return os << std::setprecision(cu::HALF_PRECISION)
            << "heading (rad): " << head.value;
}

std::ostream& operator<<(std::ostream& os, const cu::AdcData& adc) {
  return os << "ADC data: " << adc.values.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::GpsData& gps) {
  return os << "GPS data: "
            << std::setprecision(cu::FULL_PRECISION) /* precision */
            << "\n- Lat: " << gps.lat                /* latitude */
            << "\n- Lon: " << gps.lon                /* longitude */
            << "\n- Alt: " << gps.alt << std::endl;  /* altitude */
}

std::ostream& operator<<(std::ostream& os, const cu::Vec3Data& vec) {
  return os << vec.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::AccelData& accel) {
  return os << "Accel XYZ(m/s^2): " << accel.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::GyroData& gyro) {
  return os << "Gyro  XYZ(rad/s): " << gyro.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::MagData& mag) {
  return os << "Mag Field XYZ(uT): " << mag.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::RPYData& rpy) {
  return os << "TaitBryan RPY(rad): " << rpy.data.transpose();
}

std::ostream& operator<<(std::ostream& os, const cu::QuatData& quat) {
  return os << "Quat WXYZ: " << quat.data;
}

std::ostream& operator<<(std::ostream& os, const cu::ImuData& imu) {
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
