// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_STREAM_HPP_
#define CORE_UTILS_LOGGER_STREAM_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "core/utils/labeld_modifier.hpp"
#include "core/utils/logger.hpp"

namespace core::utils {

/**
 * @brief Logger Class used to log with stream method
 * @details When class is destructed it will log the stream data by calling the
 * logger function, this might throw an error depend on the the passed
 * LabeledModifier object
 */
class StreamLogger {
 public:
  // define end type for stream
  using EndlType = std::ostream&(std::ostream&);

  /**
   * @brief Construct a new Stream Logger object
   *
   * @param logger shared_ptr logger to use at destruction to log data
   * @param lm labeled modifier indicate level type and modifier
   * @param msg initial msg to log
   */
  StreamLogger(std::shared_ptr<Logger> logger, const LabeledModifier& lm,
               std::string_view init_msg = "");

  /**
   * @brief Destroy the Stream Logger object
   *
   * @throws Logger::ExceptionFactory by calling the log function
   */
  ~StreamLogger() noexcept(false);

  /**
   * @brief template overriding stream operator
   *
   * @tparam T any type
   * @param obj object to be logged
   * @return StreamLogger& return reference of class to continue logging
   */
  template <typename T>
  StreamLogger& operator<<(T obj) {
    oss_ << obj;
    return *this;
  }

  /**
   * @brief to log endl object
   *
   */
  StreamLogger& operator<<(EndlType endl);

 private:
  /// @brief shared_ptr to Logger used at destruction to log stream
  std::shared_ptr<Logger> logger_;

  /// @brief Labeled modifier used when calling Logger::Log function
  LabeledModifier lm_;

  /// @brief A stream to collect all data to be logged
  std::ostringstream oss_;
};

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_STREAM_HPP_
