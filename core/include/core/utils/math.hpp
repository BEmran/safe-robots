#ifndef CORE_UTILS_MATH_HPP
#define CORE_UTILS_MATH_HPP

#include <eigen3/Eigen/Dense>
#include <iomanip>
// #include <sstream>

namespace core::utils
{
constexpr int PRECISION = 10;
typedef float TYPE;

typedef Eigen::Matrix<TYPE, 3, 1> Vec3;
typedef Eigen::Transpose<const Vec3> Transpose;
typedef Eigen::Matrix<TYPE, 3, 3> Mat3;
typedef Eigen::Rotation2D<TYPE> Rot2;
typedef Eigen::Quaternion<TYPE> Quat;
typedef Eigen::Transform<TYPE, 3, Eigen::Affine> Transform;

Eigen::IOFormat MatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
                       "]");
Eigen::IOFormat VecFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");

std::ostream& operator<<(std::ostream& os, const Vec3& vec)
{
  return os << vec.format(VecFmt);
}

std::ostream& operator<<(std::ostream& os, const Transpose& vec)
{
  return os << vec.format(VecFmt);
}

std::ostream& operator<<(std::ostream& os, const Quat& quat)
{
  os << "ang = " << quat.w() << ", "           /* angle */
     << quat.vec().transpose().format(VecFmt); /* axis */
  return os;
}

}  // namespace core::utils

#endif  // CORE_UTILS_MATH_HPP