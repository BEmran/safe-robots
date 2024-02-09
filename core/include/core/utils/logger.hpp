// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_HPP_
#define CORE_UTILS_LOGGER_HPP_

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "core/utils/exception.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/logger_helper.hpp"
#include "core/utils/modifier.hpp"

namespace core::utils {

/**
 * @brief Return overall logger level
 *
 * @return constexpr EventLevel current overall logger level
 */
EventLevel OverallLoggerLevel();

/**
 * @brief Set the Overall Logger Level object
 *
 * @param level new overall logger level
 */
void SetOverallLoggerLevel(EventLevel level);

/**
 * @brief logger configuration used to struct Logger object
 *
 */
struct LoggerConfig {
  /// @brief Logger name
  std::string name{};

  /// @brief Writer formatter pair
  std::vector<WriterFormatterPair> wf_pairs;

  /// @brief Logging level
  EventLevel level{EventLevel::DEBUG};

  /// @brief Exception generator
  std::shared_ptr<ExceptionFactory> expectation_factory =
    std::make_shared<ExceptionFactory>(NullExceptionFactory());

  /// @brief Logger labeled modifiers
  LoggerLabeledModifiers labeled_modifiers;

  /// @brief string appended at the end of msg when logging data
  std::string end_str{"\n"};
};

/**
 * @brief A simple class used to mimic a stream to record all sort of
 * information
 */
class Logger {
 public:
  /**
   * @brief Construct the Logger object
   *
   * @param config logger configuration structure
   */
  explicit Logger(const LoggerConfig& config);

  /**
   * @brief Destroy the Logger object
   *
   */
  virtual ~Logger() = default;

  /**
   * @brief Log the passed message using the passed LabeledModifier
   *
   * @param lm label modifier to use with the formatter
   * @param msg msg to log
   */
  virtual void Log(const LabeledModifier& lm, std::string_view msg) const;

  /**
   * @brief Log the passed message using the internal LabeledModifier and passed
   * EventLevel
   *
   * @param event event level of the desired msg to log
   * @param msg msg to log
   */
  void Log(EventLevel event, std::string_view msg) const;

 protected:
  /**
   * @brief Logging implementation called by Log() function
   *
   * @param lm label modifier to use with the formatter
   * @param msg msg to be logged
   */
  void LogImp(const LabeledModifier& lm, std::string_view msg) const;

  /**
   * @brief Write message using the passed formatter and writer
   *
   * @param wf writer and formatter pair
   * @param lm label modifier to use with the formatter
   * @param msg msg to be written
   */
  void FormatAndWrite(const WriterFormatterPair& wf, const LabeledModifier& lm,
                      std::string_view msg) const;

  /**
   * @brief Throw error msg using ExceptionFactory if event is Critical
   *
   * @param event event level of the desired msg to log
   * @param msg msg to throw
   */
  void ThrowExceptionForCriticalEvent(EventLevel event,
                                      std::string_view msg) const;
  /**
   * @brief Compare LoggerLevel with OverallLoggerLevel
   *
   * @return true if LoggerLevel is bigger or equal
   * @return false otherwise
   */
  bool IsLoggerLevelSufficientToLog() const;

  /**
   * @brief Compare EventLevel with result of IsLoggerLevelSufficientToLog()
   *
   * @param EventLevel event level of the desired msg to log
   * @return true if EventLevel is bigger or equal
   * @return false otherwise
   */
  bool IsEventLevelSufficientToLog(EventLevel event) const;

 private:
  /// @brief Logger name surrounded by brackets "[]" if defined
  std::string printed_name_{};

  /// @brief Logging level
  EventLevel logging_level_{EventLevel::DEBUG};

  /// @brief Writer and formatter pairs
  std::vector<WriterFormatterPair> writer_formatter_vec_{};

  /// @brief Exception factory generator
  std::shared_ptr<ExceptionFactory> expectation_factory_;

  /// @brief Internal labeled modifier
  LoggerLabeledModifiers labeled_modifiers_;

  /// @brief string to append at the end of message before logging
  std::string end_msg_str_{};
};

/**
 * @brief Create a new Logger object with ExceptionFactory and the passed
 * stream Writer paired with TimeFormatter
 *
 * @param name name of Logger and Exception factory header
 * @param os output stream with default value as console stream
 * @return Logger logger object
 */
Logger CreateStreamLogger(std::string_view name, std::ostream& os = std::cout);

/**
 * @brief Create a new Logger object with ExceptionFactory and the passed
 * FileWriter paired with TimeFormatter
 *
 * @param name name of Logger and Exception factory header, also used to
 * create logger filename as "<name>_logger.txt" if filename is empty
 * @param filename logger file name
 * @return Logger logger object
 */
Logger CreateFileLogger(std::string_view name, std::string_view filename = "");

/**
 * @brief Create a new Logger object with ExceptionFactory and two Writers
 * (Stream and File) each paired with TimeFormatter
 *
 * @param name name of Logger and Exception factory header, also used to
 * create logger filename as "<name>_logger.txt"
 * @param os output stream with default value as console stream
 * @param filename logger file name
 * @return Logger logger object
 */
Logger CreateStreamAndFileLogger(std::string_view name,
                                 std::ostream& os = std::cout,
                                 std::string_view filename = "");

/**
 * @brief Get System logger which is a static object created with
 * CreateStreamAndFileLogger using "sys" as name and std::cout as ostream
 *
 * @return std::shared_ptr<Logger> shared_ptr to system logger
 */
std::shared_ptr<Logger> SystemLogger();
}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HPP_
