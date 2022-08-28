// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_UTILS_MATH_HPP_
#define CORE_INCLUDE_CORE_UTILS_MATH_HPP_

#include <eigen3/Eigen/Dense>
#include <iomanip>
#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE size_t
namespace core::utils {
constexpr float GRAVITY = 9.80665F;
constexpr float PI = 3.14159F;
constexpr float PI_2 = 1.570796F;

typedef float MATH_TYPE;

typedef Eigen::Matrix<MATH_TYPE, 3, 1> Vec3;
typedef Eigen::Transpose<const Vec3> Transpose;
typedef Eigen::Matrix<MATH_TYPE, 3, 3> Mat3;
typedef Eigen::Rotation2D<MATH_TYPE> Rot2;
typedef Eigen::Quaternion<MATH_TYPE> Quat;
typedef Eigen::Transform<MATH_TYPE, 3, Eigen::Affine> Transform;

}  // namespace core::utils

// need to be unscoped in namespace to be called from any file when include
// header
std::ostream& operator<<(std::ostream& os, const core::utils::Vec3& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Transpose& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Quat& quat);

#endif  // CORE_INCLUDE_CORE_UTILS_MATH_HPP_
