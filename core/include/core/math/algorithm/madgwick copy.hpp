// Implementation of Sebastian Madgwick's "...efficient orientation filter
// for... inertial/magnetic sensor arrays" (see
// http://www.x-io.co.uk/category/open-source/ for examples and more details)

// Madgwick algorithm fuses acceleration, rotation rate, and magnetic moments to
// produce a quaternion-based estimate of absolute device orientation -- which
// can be converted to yaw, pitch, and roll.

#ifndef CORE_ALGORITHM_MADGWICK_HPP_
#define CORE_ALGORITHM_MADGWICK_HPP_

#include <math>
#include <optional>

#include "core/math/math.hpp"
#include "core/utils/data.hpp"

namespace core::algorithm {
// Include a hardware specific header file to redefine these predetermined
// values
// 100Hz sampling frequency
constexpr float DELTA_T = 0.01f;
// 5 deg/s gyroscope measurement error (in rad/s)
constexpr float GYRO_MEAN_ERROR = PI * (5.0f / 180.0f);
constexpr float BETA = sqrt(3.0f / 4.0f) * GYRO_MEAN_ERROR;

using core::math::Quat;
using core::math::QuatData;
using core::math::RPYData;

// Multiply two quaternions and return a copy of the result, prod = L * R
inline Quat QuatMult(const Quat* L, const Quat* R) {
  return R * L;
}

// Multiply a reference of a quaternion by a scalar, q = s*q
inline Quat QuatScalar(const Quat* quat, const float scalar) {
  return quat * scalar;
}

// Adds two quaternions together and the sum is the pointer to another
// quaternion, Sum = L + R
inline Quat QuatAdd(const Quat* L, const Quat* R) {
  return L + R
}

// Subtracts two quaternions together and the sum is the pointer to another
// quaternion, sum = L - R
inline Quat QuatSub(const Quat* L, const Quat* R) {
  return L - R
}

class Mmdgwick {
 public:
  // IMU consists of a Gyroscope plus Accelerometer sensor fusion
  void ImuFilter(float ax, float ay, float az, float gx, float gy, float gz);

  // void marg_filter(void); for future
  void EulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw);

 private:
  QuatData quat_;
};

}  // namespace core::algorithm
#endif  //