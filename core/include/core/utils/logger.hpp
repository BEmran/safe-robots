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
#include "core/utils/formatter2.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/modifier.hpp"
#include "core/utils/writer.hpp"
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
  std::shared_ptr<FormaterInterface> formatter;
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
  virtual void Log(const LabeledModifier& lm, std::string_view msg) const;

  /**
   * @brief logs the passed message using the writer
   *
   * @param event logging label
   * @param msg msg to log
   */
  void Log(const EventLevel event, std::string_view msg) const;

  template <typename T>
  const Logger& operator<<(const T& data) const {
    for (auto& wf : writer_formatter_vec_) {
      auto& writer = *(wf.writer.get());
      writer << data;
    }
    return *this;
  }

  // /**
  //  * @brief log the passed message using the Debug LabeledModifier
  //  *
  //  * @param msg msg to log
  //  */
  // void Debug(std::string_view msg) const;

  // /**
  //  * @brief log the passed message using the Error LabeledModifier
  //  *
  //  * @param msg msg to log
  //  */
  // void Error(std::string_view msg) const;

  // /**
  //  * @brief log the passed message using the Fatal LabeledModifier
  //  *
  //  * @param msg msg to log
  //  */
  // void Fatal(std::string_view msg) const;

  // /**
  //  * @brief log the passed message using the Info LabeledModifier
  //  *
  //  * @param msg msg to log
  //  */
  // void Info(std::string_view msg) const;

  // /**
  //  * @brief log the passed message using the Warn LabeledModifier
  //  *
  //  * @param msg msg to log
  //  */
  // void Warn(std::string_view msg) const;

 protected:
  void LogImp(const EventLevel event, const std::string& event_str,
              std::string_view msg) const;

  void Dump(const WriterFormatterPair& wf, const std::string& msg) const;

  void ThrowExceptionForErrorEvent(EventLevel event,
                                   std::string_view msg) const;

 private:
  std::vector<WriterFormatterPair> writer_formatter_vec_;
  std::shared_ptr<ExceptionFactory> expectation_factory_;
  std::string name_ = "";
};

/**
 * @brief Create a new Logger object with typical console and file
 * Formatters and ExceptionFactory
 *
 * @param name name of Exception factory header, also used to create logger
 * filename as "<name>_logger.txt"
 * @return Logger logger object
 */
Logger CreateFileAndConsoleLogger(std::string_view name,
                                  std::string_view filename = "");

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HPP_
