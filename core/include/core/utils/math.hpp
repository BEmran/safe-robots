#ifndef CORE_UTILS_MATH_HPP
#define CORE_UTILS_MATH_HPP

#include <eigen3/Eigen/Dense>
#include <sstream>

namespace core::utils
{

typedef float TYPE;

typedef Eigen::Matrix<TYPE, 3, 1> Vec3;
typedef Eigen::Matrix<TYPE, 3, 3> Mat3;
typedef Eigen::Rotation2D<TYPE> Rot2;
typedef Eigen::Quaternion<TYPE> Quat;
typedef Eigen::Transform<TYPE, 3, Eigen::Affine> Transform;

Eigen::IOFormat MatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
Eigen::IOFormat VecFmt(Eigen::FullPrecision, 0, ", ", "\n", "", "", "[", "]");

std::string VecToString(const Vec3& vec)
{
    std::stringstream ss;
    ss << vec.format(VecFmt);
    return ss.str();
}

std::string MatToString(const Mat3& mat)
{
    std::stringstream ss;
    ss << mat.format(MatFmt);
    return ss.str();
}

std::string QuatToString(const Quat& quat)
{
    std::stringstream ss;
    ss << "ang = " << quat.w() << ", vec = " << quat.vec().format(VecFmt);
    return ss.str();
}

void Zero(Vec3 &vec) {
    vec.setZero();
}

void Identity(Quat &quat) {
    quat.setIdentity();
}

}  // namespace core::math

#endif  // CORE_UTILS_MATH_HPP