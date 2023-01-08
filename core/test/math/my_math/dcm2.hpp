#ifndef CORE_MATH_DCM2_HPP_
#define CORE_MATH_DCM2_HPP_

#include "matx.hpp"

/**
 * @brief Class to represent a Direction Cosine Rotation.It is built from a 3x3
 * array, but it can also be built from 3 - dimensional vectors representing the
 * *roll - pitch - yaw angles, a quaternion, or an axis - angle pair
 * representation.
 */
class DCM2 : public MatX<3> {
 public:
  DCM2() : MatX() {
  }

  DCM2(const MatType& matrix) : MatX<3>{matrix} {
  }

  /**
   * @brief Logarithm of DCM. The logarithmic map is defined as the inverse of
   * the exponential map. It corresponds to the logarithm given by the Rodrigues
   * rotation formula.
   *
   * @return Mat3 Logarithm of DCM
   */
  DCM2 Log() const {
    const float angle = std::acos((Trace() - 1.f) / 2.f);
    const MatType s = mat - T().Matrix();
    // Skew - symmetric matrix
    const MatType logR = angle * s / (2.f * std::sin(angle));
    return logR;
  }

  /**
   * @brief Return the adjugate of the DCM
   *
   * @return Mat3 adjugate matrix of DCM
   */
  DCM2 Adjugate() const {
    return MatType(T().Matrix() * Det());
  }
};

bool AssertSO3(const DCM2 dcm) {
  const float eps = 0.0001f;
  const bool det_of_1 = std::abs(dcm.Det() - 1) < eps;
  const bool orthogonal = std::abs((dcm.Matrix() * dcm.T()).trace() - 3) < eps;
  return det_of_1 && orthogonal;
}
#endif  // CORE_MATH_DCM2_HPP_