#ifndef CORE_MATH_TRANSFORMATION_HPP_
#define CORE_MATH_TRANSFORMATION_HPP_

#include "core/utils/math.hpp"
#include "dcm.hpp"
#include "quaternion.hpp"

namespace my {
using core::utils::Mat3;
using core::utils::RPY;
using core::utils::Vec3;

enum class EulerOrder { XYZ, ZYX };
enum class QuaternionMethod { SHEPPERD, SARABANDI, CHIAVERINI };

template <typename T>
T Sign(const T val) {
  return static_cast<T>((T(0) < val) - (val < T(0)));
}

template <typename T>
T Square(const T val) {
  return val * val;
}

/**
 * @brief Construct a skew-symmetric matrix, a square matrix whose transpose
 * equals its negative
 *
 * @param vec vector to be used
 * @return Mat3 skew skew-symmetric matrix
 */
Mat3 Skew(const Vec3 vec);

/**
 * @brief Construct a DCM from axis-angle representation. Use
 * Rodrigue's formula to obtain the DCM from the axis-angle representation.
 *
 * @return DCM dcm matrix
 */
DCM AxisAngleToDCM(const float angle, const Vec3 axis);

/**
 * @brief  Return axis-angle representation of the DCM.
 * @details The axis-angle representation is not unique
 *
 * @return std::pair<float, Vec3> Angle of rotation [rad], Axis of rotation.
 */
std::pair<float, Vec3> DCMToAxisAngle(const DCM& dcm);

DCM EulerXYZToDCM(const RPY rpy);
DCM EulerZYXToDCM(const RPY rpy);
DCM EulerToDCM(const RPY rpy, const EulerOrder order);

/**
 * @brief Roll-Pitch-Yaw Angles from DCM
 *
 * @return Vec3 a vector with [roll, pitch, yaw] angles
 */
RPY DCMToEulerXYZ(const DCM dcm);
RPY DCMToEulerZYX(const DCM dcm);
RPY DCMToEuler(const DCM& dcm, const EulerOrder order);

/**
 * @brief Quaternion from a Direction Cosine Matrix with Shepperd's method
 *
 * @param dcm Direction Cosine Matrix
 * @return Quaternion quaternion
 */
Quaternion Shepperd2(const DCM dcm);
Quaternion Shepperd(const DCM dcm);

/**
 * @brief Quaternion from a Direction Cosine Matrix with Sarabandi's method.
 *
 * @param dcm Direction Cosine Matrix
 * @return Quaternion quaternion
 */
Quaternion Sarabandi(const DCM dcm);

/**
 * @brief Quaternion from a Direction Cosine Matrix with Chiaverini's algebraic
 * method.
 *
 * @param dcm Direction Cosine Matrix
 * @return Quaternion quaternion
 */
Quaternion Chiaverini(const DCM dcm);

Quaternion DCMToQuaternion(
  const DCM& dcm, const QuaternionMethod method = QuaternionMethod::SHEPPERD);

/**
 * @brief Construct a DCM from quaternion vector
 *
 * @param quat quaternion vector
 * @return DCM rotation matrix
 */
DCM QuatToDCM(const Quaternion quat);
}  // namespace my
#endif  // CORE_MATH_TRANSFORMATION_HPP_