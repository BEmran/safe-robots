#include "core/utils/data.hpp"

namespace core::utils
{
std::ostream& operator<<(std::ostream& os, const Vec3& vec3)
{
  os << std::setprecision(10)       /* set precision to 10 digits */
     << "vec = [" << vec3 << "]"; /* value of x-axis */
  return os;
}

std::ostream& operator<<(std::ostream& os, const Quat& quat)
{
  os << std::setprecision(10)           /* set precision to 10 digits */
     << "angle = " << quat.w() << ", "  /* value of rotation angle */
     << "vec = [" << quat.vec() << "]"; /* vector value */

  return os;
}
}  // namespace core::utils