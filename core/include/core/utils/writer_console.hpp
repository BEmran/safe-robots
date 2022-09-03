// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_WRITER_CONSOLE_HPP_
#define CORE_UTILS_WRITER_CONSOLE_HPP_

#include <string>

#include "core/utils/writer.hpp"

namespace core::utils {
/**
 * @brief A concrete class of Writer used to writer data to the console
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
  ~ConsoleWriter() override = default;

  /* Writer Interface */
  void Dump(const std::string& str) const override;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITER_CONSOLE_HPP_
