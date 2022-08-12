#include "core/utils/math.hpp"

namespace core::utils
{

Eigen::IOFormat MatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[",
                       "]");
Eigen::IOFormat VecFmt(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[", "]");

std::ostream& operator<<(std::ostream& os, const Vec3& vec)
{
  return os << vec.format(VecFmt);
}

std::ostream& operator<<(std::ostream& os, const Transpose& vec)
{
  return os << vec.format(VecFmt);
}

std::ostream& operator<<(std::ostream& os, const Quat& quat)
{
  os << "ang = " << quat.w() << ", "           /* angle */
     << quat.vec().transpose().format(VecFmt); /* axis */
  return os;
}

}  // namespace core::utils