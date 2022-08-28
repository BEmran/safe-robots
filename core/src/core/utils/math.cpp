#include "core/utils/math.hpp"

namespace core::utils
{
static const Eigen::IOFormat kMatFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[",
                                     "]", "[", "]");
static const Eigen::IOFormat kVecFmt(Eigen::FullPrecision, 0, ", ", ";\n", "",
                                     "", "[", "]");

std::ostream& operator<<(std::ostream& os, const Vec3& vec)
{
  return os << vec.format(kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const Transpose& vec)
{
  return os << vec.format(kVecFmt);
}

std::ostream& operator<<(std::ostream& os, const Quat& quat)
{
  os << "ang = " << quat.w() << ", "            /* angle */
     << quat.vec().transpose().format(kVecFmt); /* axis */
  return os;
}

}  // namespace core::utils