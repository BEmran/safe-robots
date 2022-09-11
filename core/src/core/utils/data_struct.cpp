// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/data_struct.hpp"

namespace core::utils {

std::ostream& operator<<(std::ostream& os,
                         const DataStructInterface* const dsi) {
  return os << "[" << dsi->Header() << "]: " << dsi->ToString();
}

std::ostream& operator<<(std::ostream& os, const Imu& imu) {
  std::for_each(imu.array.begin(), imu.array.end() - 1,
                [&os](DataStructInterface* const dsi) { os << dsi << ", "; });
  os << imu.array.back();
  return os;
}
}  // namespace core::utils
