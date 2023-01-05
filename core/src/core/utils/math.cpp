// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/utils/math.hpp"

namespace core::utils {
static const Eigen::IOFormat kMatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[",
                                     "]", "[", "]");
static const Eigen::IOFormat kVecFmt(Eigen::FullPrecision, 0, ", ", ";\n", "",
                                     "", "[", "]");

// MatrixX CreateMatrix(int r, int c) {
//   MatrixX mat;
//   mat.resize(r, c);
//   return mat;
// }

// MatrixX CreateVector(int size) {
//   return CreateMatrix(size, 1);
// }

Scalar CreateScalar(MATH_TYPE val) {
  Scalar s;
  s << val;
  return s;
}

Quat UnitQuaternion(const float scalar, const Vec3 vec) {
  Quat q(scalar, vec.x(), vec.y(), vec.z());
  q.normalize();
  return q;
}

}  // namespace core::utils

std::ostream& operator<<(std::ostream& os, const core::utils::Vec3& vec) {
  return os << vec.format(core::utils::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::utils::Transpose& vec) {
  return os << vec.format(core::utils::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::utils::Quat& quat) {
  os << "ang = " << quat.w() << ", "                         /* angle */
     << quat.vec().transpose().format(core::utils::kVecFmt); /* axis */
  return os;
}
