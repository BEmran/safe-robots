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

class StreamLogger;

/**
 * @brief logger configuration used to struct Logger object
 *
 */
struct LoggerConfig {
  std::string name = "";                      // logger name
  std::vector<WriterFormatterPair> wf_pairs;  // writer formatter pair
  EventLevel level = EventLevel::DEBUG;       // logging level
  std::shared_ptr<ExceptionFactory> expectation_factory =
    std::make_shared<ExceptionFactory>(NullExceptionFactory());  // exception
                                                                 // generator
  LoggerLabeledModifiers labeled_modifiers;  // logger labeled modifiers
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
   * @brief Log the passed message using the Debug LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Debug(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Error LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Error(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Fatal LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Fatal(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Info LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Info(std::string_view msg = "") const;

  /**
   * @brief Log the passed message using the Warn LabeledModifier
   *
   * @param msg msg to log
   * @return StreamLogger logger used with stream to log extra data
   */
  StreamLogger Warn(std::string_view msg = "") const;

  /**
   * @brief logs the passed message using the writer
   *
   * @param lm label modifier to use with the formatter
   * @param msg msg to log
   */
  virtual void Log(const LabeledModifier& lm, std::string_view msg) const;

  /**
   * @brief logs the passed message using the writer
   *
   * @param event logging label
   * @param msg msg to log
   */
  void Log(const EventLevel event, std::string_view msg) const;

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
   * @param event event level
   * @param msg msg to throw
   */
  void ThrowExceptionForCriticalEvent(EventLevel event,
                                      std::string_view msg) const;

 private:
  // logger name surrounded by brackets "[]" if defined
  std::string printed_name_{};
  // Logging level
  EventLevel logging_level_{EventLevel::DEBUG};
  // writer and formatter pairs
  std::vector<WriterFormatterPair> writer_formatter_vec_{};
  // Exception factory generator
  std::shared_ptr<ExceptionFactory> expectation_factory_;
  // Internal labeled modifier
  LoggerLabeledModifiers labeled_modifiers_;
};

class StreamLogger {
 public:
  using endl_type = std::ostream&(std::ostream&);

  StreamLogger(const Logger& logger, const LabeledModifier& lm,
               std::string_view msg = "")
    : logger_(logger), lm_(lm) {
    oss_ << msg;
  }

  ~StreamLogger() noexcept(false) {
    // std::stringstream ss;
    oss_ << "\n";
    logger_.Log(lm_, oss_.str());  // cppcheck-suppress exceptThrowInDestructor
  }

  template <typename T>
  StreamLogger& operator<<(T obj) {
    oss_ << obj;
    return *this;
  }

  StreamLogger& operator<<(endl_type endl) {
    oss_ << endl;
    return *this;
  }

 private:
  const Logger& logger_;
  LabeledModifier lm_;
  std::ostringstream oss_;
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

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HPP_
