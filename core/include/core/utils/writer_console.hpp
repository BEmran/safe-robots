// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_
#define CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_

#include <string>

#include "core/utils/writer.hpp"

namespace core::utils {
/**
 * @brief A concreate class of Writer used to log data to the console
 * using ostream object.
 *
 */
class ConsoleWriter : public Writer {
 public:
  /**
   * @brief Construct a new Console Writer object with a specific name
   *
   */
  ConsoleWriter() = default;

  /**
   * @brief Destroy the Console Writer object
   *
   */
  ~ConsoleWriter() = default;

  /* Writer Interface */
  void Dump(const std::string& str) override;
};

}  // namespace core::utils

#endif  // CORE_INCLUDE_CORE_UTILS_WRITTER_CONSOLE_HPP_
