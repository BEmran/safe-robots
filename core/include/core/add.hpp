#ifndef __CORE_ADD_HPP__
#define __CORE_ADD_HPP__

#include <tuple>
#include <vector>

namespace core
{
/**
 * @brief Accumulate a vector to produce the sum and mean
 *
 * @param vec the vector of values
 * @return std::tuple<double, double> sum and mean of a vector of double values.
 */
std::tuple<double, double> accumulate_vector(const std::vector<double>& vec);
}  // namespace core

#endif  // __CORE_ADD_HPP__