// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_WRITTER_HPP_
#define CORE_UTILS_WRITTER_HPP_

#include <string>

namespace core::utils {
class Writer {
 public:
  /**
   * @brief Destroy the Writer object
   *
   */
  virtual ~Writer() = default;

  virtual void Dump(const std::string& str) = 0;
};

}  // namespace core::utils

#endif  // CORE_UTILS_WRITTER_HPP_
