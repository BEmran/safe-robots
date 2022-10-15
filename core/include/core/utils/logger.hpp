// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_HPP_
#define CORE_UTILS_LOGGER_HPP_

#include <cstring>  // to use strrchr
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "core/utils/exception.hpp"
#include "core/utils/formatter.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/modifier.hpp"
#include "core/utils/writer.hpp"
#include "core/utils/writer_file.hpp"

// extract filename from file's full path
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

namespace core::utils {

class NestedLogger;

/**
 * @brief location information about where the log came from
 *
 */
struct LogLocation {
  std::string file;
  std::string func;
  int line;
  /**
   * @brief create a new LogLocation object
   *
   */
  LogLocation(const char* file_, const char* func_, int line_)
    : file(file_), func(func_), line(line_) {
  }

  /**
   @brief Converts object information to string in the format of
   * [file][func][line]
   *
   * @return std::string a string contains the object information
   */
  inline std::string ToString() const {
    return "[" + file + "][" + func + "][" + std::to_string(line) + "]";
  }
};

/**
 * @brief a macro to create LogLocation object
 *
 */
#define LOG_INFORMATION                                                        \
  core::utils::LogLocation(__FILENAME__, __func__, __LINE__)

/**
 * @brief a macro used to simplify logging LogLocation object
 *
 */
#define LOG_INFORMATION_STRING LOG_INFORMATION.ToString()

/**
 * @brief simple struct to pair a writer with formatter
 * TODO(bara): do we need shared_ptr?
 */
struct WriterFormatterPair {
  std::shared_ptr<Writer> writer = nullptr;
  Formatter formatter = CreateNullFormatter();
};

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
  friend NestedLogger;
  /**
   * @brief log data on all Writers using stream method.
   * @details This logging method is used for continues logging thus it does not
   * use formatter here
   *
   * @tparam T anytype
   * @param data data to be logged
   * @return const Logger& constant reference to the logger
   */
  template <typename T>
  const Logger& operator<<(const T& data) const {
    for (size_t idx = 0; idx < writer_formatter_vec_.size(); ++idx) {
      auto writer_shared = writer_formatter_vec_[idx].writer;
      if (writer_shared) {
        auto& writer = *(writer_shared.get());
        writer << data;
      } else {
        std::cerr << "Logger: Writer #[" << idx << "] is invalid" << std::endl;
      }
    }
    return *this;
  }

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
  /**
   * @brief formatter name surrounded by brackets "[]" if defined
   *
   */
  std::string printed_name_{};

  /**
   * @brief Logging level
   *
   */
  EventLevel logging_level_{EventLevel::DEBUG};

  /**
   * @brief writer and formatter pairs
   *
   */
  std::vector<WriterFormatterPair> writer_formatter_vec_{};

  /**
   * @brief Exception factory generator
   *
   */
  std::shared_ptr<ExceptionFactory> expectation_factory_;
};

class NestedLogger {
 public:
  NestedLogger(const Logger& logger) : logger_(logger) {
  }

  ~NestedLogger() {
    logger_ << "\n";
  }

  template <typename T>
  const NestedLogger& operator<<(const T& data) const {
    logger_ << data;
    return *this;
  }

 private:
  const Logger& logger_;
};

/**
 * @brief Create a new Logger object with ExceptionFactory and the passed stream
 * Writer paired with TimeFormatter
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
 * @param name name of Logger and Exception factory header, also used to create
 * logger filename as "<name>_logger.txt" if filename is empty
 * @param filename logger file name
 * @return Logger logger object
 */
Logger CreateFileLogger(std::string_view name, std::string_view filename = "");

/**
 * @brief Create a new Logger object with ExceptionFactory and two Writers
 * (Stream and File) each paired with TimeFormatter
 *
 * @param name name of Logger and Exception factory header, also used to create
 * logger filename as "<name>_logger.txt"
 * @param os output stream with default value as console stream
 * @param filename logger file name
 * @return Logger logger object
 */
Logger CreateStreamAndFileLogger(std::string_view name,
                                 std::ostream& os = std::cout,
                                 std::string_view filename = "");

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HPP_
