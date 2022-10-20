// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/data_struct.hpp"

namespace core::utils {

std::ostream& operator<<(std::ostream& os,
                         const DataStructInterface* const dsi) {
  return os << "[" << dsi->Header() << "]: " << dsi->ToString();
}

std::ostream& operator<<(std::ostream& os, const Imu& imu) {
  os << "IMU:";
  for (auto element : imu.array) {
    os << "\n\t -" << element;
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const ImuDataStruct& imu_data) {
  return os << imu_data.Label() << ": " << imu_data.Get();
}
}  // namespace core::utils
