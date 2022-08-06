#include "core/add.hpp"

#include <numeric>

namespace core
{
std::tuple<double, double> accumulate_vector(const std::vector<double>& vec)
{
  auto const sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  auto const count = static_cast<double>(vec.size());
  auto const avg = sum / count;
  return {sum, avg};
}
}  // namespace core