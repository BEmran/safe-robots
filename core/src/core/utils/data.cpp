// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/data.hpp"

namespace core::utils {

template <class T>
using CB = std::function<std::string(T)>;

template <class T>
std::string ApplyOnEach(const std::vector<T>& vec, CB<T> cb) {
  std::string str;
  const size_t size = vec.size();
  for (size_t idx = 0; idx < size; ++idx) {
    if (idx == size - 1) {
      str += cb(vec[idx]);
    } else {
      str += cb(vec[idx]) + ", ";
    }
  }
  return str;
}

std::string Header(const std::vector<Data*>& vec) {
  auto lambda = [](Data* d) -> std::string { return d->Header(); };
  return ApplyOnEach<Data*>(vec, lambda);
}

std::string Header(const std::vector<std::shared_ptr<Data>>& vec) {
  auto lambda = [](std::shared_ptr<Data> d) -> std::string {
    return d->Header();
  };
  return ApplyOnEach<std::shared_ptr<Data>>(vec, lambda);
}

std::string ToString(const std::vector<Data*>& vec) {
  auto lambda = [](Data* d) -> std::string { return d->ToString(); };
  return ApplyOnEach<Data*>(vec, lambda);
}

std::string ToString(const std::vector<std::shared_ptr<Data>>& vec) {
  auto lambda = [](std::shared_ptr<Data> d) -> std::string {
    return d->ToString();
  };
  return ApplyOnEach<std::shared_ptr<Data>>(vec, lambda);
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
