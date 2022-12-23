#ifndef CORE_ALGORITHM_MADGWICK_HPP_
#define CORE_ALGORITHM_MADGWICK_HPP_

#include <optional>

#include "core/utils/data.hpp"
#include "core/utils/math.hpp"

namespace core::algorithm {
using core::utils::Quat;
using core::utils::Vec3;

struct MadgwickConfig {
  float dt{0.01};
  float gain{0.9};
};

/**
 * @brief Attitude quaternion obtained with gyroscope and
 * accelerometer-magnetometer measurements, via complementary filter.
 *
 */
class Madgwick {
 public:
  Madgwick(const MadgwickConfig config);

  // Attitude Estimation from given measurements and previous orientation.
  std::optional<Quat> Update(const Vec3& accel, const Vec3& gyro,
                             const float dt = -1);

  // Attitude Estimation from given measurements and previous orientation.
  std::optional<Quat> Update(const Vec3& accel, const Vec3& gyro,
                             const Vec3& mag, const float dt = -1);

 protected:
  bool CheckIfVec3IsValid(const Vec3& vec) const;

 private:
  MadgwickConfig config_;
  Quat quat_{Quat().Identity()};
};

}  // namespace core::algorithm
#endif  // CORE_ALGORITHM_MADGWICK_HPP_