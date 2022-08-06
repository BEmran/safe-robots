#ifndef CORE_UTILS_WRITTER_HPP
#define CORE_UTILS_WRITTER_HPP

#include <string>

namespace core
{
namespace utils
{
class Writter
{
 public:
  /**
   * @brief Destroy the Writter object
   *
   */
  virtual ~Writter()
  {
  }

  virtual void dump(const std::string& str) = 0;
};
}  // namespace utils
}  // namespace core

#endif  // CORE_UTILS_WRITTER_HPP