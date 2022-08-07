#include "core/utils/data.hpp"

namespace core::utils
{
std::ostream& operator<<(std::ostream& os, const Vec3& vec3) {
  os << std::setprecision(10) << "[" /* set precision to 10 digits */
     << vec3.x << ", "               /* value of x-axis */
     << vec3.y << ", "               /* value of y-axis */
     << vec3.z << "]";               /* value of z-axis */
  return os;
}

std::ostream& operator<<(std::ostream& os, const Quat& quat) {
  os << std::setprecision(10) << "[" /* set precision to 10 digits */
     << quat.w << ", "               /* value of rotation angle */
     << quat.x << ", "               /* value of x-axis */
     << quat.y << ", "               /* value of y-axis */
     << quat.z << "]";               /* value of z-axis */
  return os;
}
}  // namespace core::utils