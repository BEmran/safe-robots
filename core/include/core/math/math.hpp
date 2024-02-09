// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_MATH_MATH_HPP_
#define CORE_MATH_MATH_HPP_

#include <eigen3/Eigen/Dense>
#include <iomanip>

#undef EIGEN_DEFAULT_DENSE_INDEX_TYPE
#define EIGEN_DEFAULT_DENSE_INDEX_TYPE size_t

namespace core::math {
using MATH_TYPE = float;

constexpr MATH_TYPE GRAVITY = 9.80665F;
constexpr MATH_TYPE PI = 3.14159F;
constexpr MATH_TYPE PI_2 = PI / 2.F;
constexpr MATH_TYPE PI_4 = PI / 4.F;
constexpr MATH_TYPE DEG_TO_RAD = PI / 180.F;

using Vec3 = Eigen::Matrix<MATH_TYPE, 3, 1>;
using Mat3 = Eigen::Matrix<MATH_TYPE, 3, 3>;
using Mat4 = Eigen::Matrix<MATH_TYPE, 4, 4>;
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
Quat UnitQuaternion(float scalar, const Vec3& vec);

/**
 * @brief simple structure to hold RPY data in radian
 *
 */
struct RPY {
  /// @brief angle around x-axis in radian
  MATH_TYPE roll{0.F};
  /// @brief angle around y-axis in radian
  MATH_TYPE pitch{0.F};
  /// @brief angle around z-axis in radian
  MATH_TYPE yaw{0.F};

  /**
   * @brief Default Construct for RPY object
   *
   */
  RPY() = default;

  /**
   * @brief Construct a new RPY object
   *
   * @param r roll angle in radian
   * @param p pitch angle in radian
   * @param y yaw angle in radian
   */
  RPY(const MATH_TYPE r, const MATH_TYPE p, const MATH_TYPE y)
    : roll(r), pitch(p), yaw(y) {
  }
};
}  // namespace core::math

// need to be unscoped in namespace to be called from any file when include
// header
std::ostream& operator<<(std::ostream& os, const core::math::Vec3& vec);
std::ostream& operator<<(std::ostream& os, const core::math::Transpose& vec);
std::ostream& operator<<(std::ostream& os, const core::math::Quat& quat);
std::ostream& operator<<(std::ostream& os, const core::math::RPY& rpy);

#endif  // CORE_MATH_MATH_HPP_
