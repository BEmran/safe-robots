// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_MATH_HPP_
#define CORE_UTILS_MATH_HPP_

#include <eigen3/Eigen/Dense>
#include <iomanip>

#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE size_t

namespace core::utils {
using MATH_TYPE = float;

constexpr MATH_TYPE GRAVITY = 9.80665F;
constexpr MATH_TYPE PI = 3.14159F;
constexpr MATH_TYPE PI_2 = 1.570796F;
constexpr MATH_TYPE DEG_TO_RAD = PI / 180.F;

using Vec3 = Eigen::Matrix<MATH_TYPE, 3, 1>;
using Mat3 = Eigen::Matrix<MATH_TYPE, 3, 3>;
using Rot2 = Eigen::Rotation2D<MATH_TYPE>;
using Quat = Eigen::Quaternion<MATH_TYPE>;
using Transform = Eigen::Transform<MATH_TYPE, 3, Eigen::Affine>;
using MatrixX = Eigen::Matrix<MATH_TYPE, -1, -1>;
using Transpose = Eigen::Transpose<const Vec3>;

template <int R, int C>
using Matrix = Eigen::Matrix<MATH_TYPE, R, C>;
template <int R>
using Vector = Eigen::Matrix<MATH_TYPE, R, 1>;
using Scalar = Eigen::Matrix<MATH_TYPE, 1, 1>;
using InputMat = const Eigen::Ref<const MatrixX>&;
using OutputMat = Eigen::Ref<MatrixX>;

static const Eigen::IOFormat kVecFmtSimple(Eigen::FullPrecision, 0, ", ", ", ",
                                           "", "", "", "");

Scalar CreateScalar(MATH_TYPE val);

}  // namespace core::utils

// need to be unscoped in namespace to be called from any file when include
// header
std::ostream& operator<<(std::ostream& os, const core::utils::Vec3& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Transpose& vec);
std::ostream& operator<<(std::ostream& os, const core::utils::Quat& quat);

#endif  // CORE_UTILS_MATH_HPP_
