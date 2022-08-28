// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_
#define CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_

#include <string>

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

#endif  // CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_
