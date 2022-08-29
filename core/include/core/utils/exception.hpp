// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_EXCEPTION_HPP_
#define CORE_UTILS_EXCEPTION_HPP_

#include <stdexcept>
#include <string>
#include <utility>

namespace core::utils {
class Exception : public std::runtime_error {
 public:
  explicit Exception(const std::string& msg) : std::runtime_error(msg) {
  }
};

class ExceptionFactory {
 public:
  explicit ExceptionFactory(const std::string& header) : header_(header) {
  }

  virtual ~ExceptionFactory() = default;

  virtual void Throw(const std::string& msg) const {
    if (header_.empty()) {
      throw Exception(msg);
    }
    throw Exception(header_ + ": " + msg);
  }

 private:
  std::string header_;
};

class NullExceptionFactory : public ExceptionFactory {
 public:
  NullExceptionFactory() : ExceptionFactory("") {
  }

  void Throw(const std::string& msg) const final {
    (void)msg;
  }
};

}  // namespace core::utils

#endif  // CORE_UTILS_EXCEPTION_HPP_
