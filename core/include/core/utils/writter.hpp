// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_INCLUDE_CORE_UTILS_WRITTER_HPP_
#define CORE_INCLUDE_CORE_UTILS_WRITTER_HPP_

#include <string>

namespace core::utils {
class Writter {
 public:
  /**
   * @brief Destroy the Writter object
   *
   */
  virtual ~Writter() {
  }

  virtual void dump(const std::string& str) = 0;
};

}  // namespace core::utils

#endif  // CORE_INCLUDE_CORE_UTILS_WRITTER_HPP_
