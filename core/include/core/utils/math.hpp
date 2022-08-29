// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_MATH_HPP_
#define CORE_UTILS_MATH_HPP_

#include <eigen3/Eigen/Dense>
#include <iomanip>

#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE size_t

namespace core::utils {
constexpr float GRAVITY = 9.80665F;
constexpr float PI = 3.14159F;
constexpr float PI_2 = 1.570796F;
constexpr float RAD_TO_DEG = PI / 180.F;
typedef float MATH_TYPE;

using Vec3 = Eigen::Matrix<MATH_TYPE, 3, 1>;
using Transpose = Eigen::Transpose<const Vec3>;
using Mat3 = Eigen::Matrix<MATH_TYPE, 3, 3>;
using Rot2 = Eigen::Rotation2D<MATH_TYPE>;
using Quat = Eigen::Quaternion<MATH_TYPE>;
using Transform = Eigen::Transform<MATH_TYPE, 3, Eigen::Affine>;

}  // namespace core::utils

// need to be unscoped in namespace to be called from any file when include
// header
std::ostream& operator<<(std::ostream& os, const core::utils::Vec3& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Transpose& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Quat& quat);

#endif  // CORE_UTILS_MATH_HPP_
