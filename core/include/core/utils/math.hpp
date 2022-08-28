#ifndef CORE_UTILS_MATH_HPP
#define CORE_UTILS_MATH_HPP

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

std::ostream& operator<<(std::ostream& os, const Vec3& vec);

std::ostream& operator<<(std::ostream& os, const Transpose& vec);

std::ostream& operator<<(std::ostream& os, const Quat& quat);

}  // namespace core::utils

#endif  // CORE_UTILS_MATH_HPP