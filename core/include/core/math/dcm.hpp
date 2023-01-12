#ifndef CORE_MATH_DCM_HPP_
#define CORE_MATH_DCM_HPP_

#include "core/math/math.hpp"

namespace core::math {
using core::math::Mat3;
using core::math::MATH_TYPE;

/**
 * @brief Class to represent a Direction Cosine Rotation.It is built from a 3x3
 * array, but it can also be built from 3 - dimensional vectors representing the
 * *roll - pitch - yaw angles, a quaternion, or an axis - angle pair
 * representation.
 */
class DCM {
 public:
  DCM();

  DCM(const Mat3& matrix);

  Mat3 Matrix() const;

  Mat3& Matrix();

  DCM Inv() const;
  void InvInPlace();

  DCM T() const;

  void TransposeInPlace();

  float Det() const;

  float Norm() const;

  float Trace() const;

  /**
   * @brief Logarithm of DCM. The logarithmic map is defined as the inverse of
   * the exponential map. It corresponds to the logarithm given by the Rodrigues
   * rotation formula.
   *
   * @return Mat3 Logarithm of DCM
   */
  Mat3 Log() const;

  /**
   * @brief Return the adjugate of the DCM
   *
   * @return Mat3 adjugate matrix of DCM
   */
  Mat3 Adjugate() const;

  MATH_TYPE operator[](const size_t idx) const;

  MATH_TYPE& operator[](const size_t idx);

  MATH_TYPE At(const size_t idx) const;

  MATH_TYPE& At(const size_t idx);

  MATH_TYPE At(const size_t row, const size_t col) const;

  MATH_TYPE& At(const size_t row, const size_t col);

 private:
  Mat3 mat;
};
}  // namespace math
#endif  // CORE_MATH_DCM_HPP_