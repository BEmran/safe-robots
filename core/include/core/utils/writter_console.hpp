#ifndef CORE_UTILS_WRITTER_CONSOLE_HPP
#define CORE_UTILS_WRITTER_CONSOLE_HPP

#include "core/utils/writter.hpp"

namespace core::utils {
/**
 * @brief A concreate class of Writter used to log data to the console
 * using ostream object.
 *
 */
class ConsoleWritter : public Writter {
 public:
  /**
   * @brief Construct a new Console Writter object with a specific name
   *
   */
  ConsoleWritter() = default;

  /**
   * @brief Destroy the Console Writter object
   *
   */
  ~ConsoleWritter() = default;

  /* Writter Interface */
  void dump(const std::string& str) override;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITTER_CONSOLE_HPP
