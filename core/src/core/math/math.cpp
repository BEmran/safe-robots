// Copyright (C) 2022 Bara Emran - All Rights Reserved

#include "core/math/math.hpp"

namespace core::math {
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

Quat UnitQuaternion(float scalar, const Vec3& vec) {
  Quat q(scalar, vec.x(), vec.y(), vec.z());
  q.normalize();
  return q;
}

}  // namespace core::math

std::ostream& operator<<(std::ostream& os, const core::math::Vec3& vec) {
  return os << vec.format(core::math::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::math::Transpose& vec) {
  return os << vec.format(core::math::kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const core::math::Quat& quat) {
  os << "ang = " << quat.w() << ", "                        /* angle */
     << quat.vec().transpose().format(core::math::kVecFmt); /* axis */
  return os;
}

std::ostream& operator<<(std::ostream& os, const core::math::RPY& rpy) {
  return os << "roll = " << rpy.roll << ", "
            << "pitch = " << rpy.pitch << ", "
            << "yaw = " << rpy.yaw;
}
