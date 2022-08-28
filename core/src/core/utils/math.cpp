// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/math.hpp"

namespace core::utils {
static const Eigen::IOFormat kMatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[",
                                     "]", "[", "]");
static const Eigen::IOFormat kVecFmt(Eigen::FullPrecision, 0, ", ", ";\n", "",
                                     "", "[", "]");
}  // namespace core::utils

std::ostream& operator<<(std::ostream& os, const core::utils::Vec3& vec) {
  return os << vec.format(core::utils::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::utils::Transpose& vec) {
  return os << vec.format(core::utils::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::utils::Quat& quat) {
  os << "ang = " << quat.w() << ", "            /* angle */
     << quat.vec().transpose().format(core::utils::kVecFmt); /* axis */
  return os;
}
