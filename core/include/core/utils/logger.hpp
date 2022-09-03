// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_HPP_
#define CORE_UTILS_LOGGER_HPP_

#include <cstring>  // to use strrchr
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "core/utils/event_level.hpp"
#include "core/utils/exception.hpp"
#include "core/utils/formatter.hpp"
#include "core/utils/terminal.hpp"
#include "core/utils/writer_console.hpp"
#include "core/utils/writer_file.hpp"

// extract filename from file's full path
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

namespace core::utils {
/**
 * @brief location information about where the log came from
 *
 */
struct LogLocation {
  std::string file;
  std::string func;
  int line;
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

#define LOG_INFORMATION                                                        \
  core::utils::LogLocation(__FILENAME__, __func__, __LINE__)
#define LOG_INFORMATION_STRING LOG_INFORMATION.ToString()

struct WriterFormatterPair {
  std::shared_ptr<Writer> writer;
  Formatter formatter;
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
   * @param writer_formatter vector of WriterFormatter pairs
   */
  explicit Logger(const std::vector<WriterFormatterPair>& writer_formatter);

  /**
   * @brief Construct the Logger object
   *
   * @param writer_formatter vector of WriterFormatter pairs
   * @param expectation_factory shared ptr to exception factory
   */
  Logger(const std::vector<WriterFormatterPair>& writer_formatter,
         std::shared_ptr<ExceptionFactory> expectation_factory);

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
  virtual void Log(const LabeledModifier& lm, const std::string& msg);

 protected:
  static void Dump(const WriterFormatterPair& wf, const LabeledModifier& lm,
                   const std::string& msg);

  void ThrowExceptionForErrorEvent(EventLevel event, const std::string& msg);

 private:
  std::vector<WriterFormatterPair> writer_formatter_vec_;
  std::shared_ptr<ExceptionFactory> expectation_factory_;
};

/**
 * @brief Create a new Logger object with typical console and file
 * Formatters and ExceptionFactory
 *
 * @param name name of Exception factory header, also used to create logger
 * filename as "<name>_logger.txt"
 * @return std::shared_ptr<Logger> logger object
 */
std::shared_ptr<Logger> CreateFileAndConsoleLogger(const std::string& name);

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HPP_
