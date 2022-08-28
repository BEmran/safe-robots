

#ifndef CORE_UTILS_TRACER_HPP
#define CORE_UTILS_TRACER_HPP

#include <string.h>  // to use strrchr

#include <iostream>
#include <memory>
#include <sstream>

#include "core/utils/event_level.hpp"
#include "core/utils/exception.hpp"
#include "core/utils/formatter.hpp"
#include "core/utils/terminal.hpp"
#include "core/utils/writter_console.hpp"
#include "core/utils/writter_file.hpp"

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
  LogLocation(const char* file_, const char* func_, const int line_)
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

/**
 * @brief A simple class used to mimic a stream to record all sort of
 * information
 */
class Logger {
 public:
  /**
   * @brief Construct the Logger object using filename
   *
   * @param filename writter file name
   */
  explicit Logger(const std::string& filename);

  /**
   * @brief Construct the Logger object using filename
   *
   * @param filename writter file name
   * @param expectation_factory shared ptr to exception factory
   */
  Logger(const std::string& filename,  //
         std::shared_ptr<ExceptionFactory> expectation_factory);

  /**
   * @brief Construct the Logger object using filename
   *
   * @param filename writter file name
   * @param file_formater file formatter to use with file writter
   * @param console_formater console formatter to use with file writter
   */
  Logger(const std::string& filename,
         std::shared_ptr<FormatterInterface> file_formater,
         std::shared_ptr<FormatterInterface> console_formater);

  /**
   * @brief Construct the Logger object using filename
   *
   * @param filename writter file name
   * @param file_formater file formatter to use with file writter
   * @param console_formater console formatter to use with file writter
   * @param expectation_factory shared ptr to exception factory
   */
  Logger(const std::string& filename,
         std::shared_ptr<FormatterInterface> file_formater,
         std::shared_ptr<FormatterInterface> console_formater,
         std::shared_ptr<ExceptionFactory> expectation_factory);

  /**
   * @brief Destroy the Logger object
   *
   */
  virtual ~Logger() {
  }

  /**
   * @brief logs the passed message using the writter
   *
   * @param lm label modifier to use with the formatter
   * @param msg msg to log
   */
  virtual void Log(const LabeledModifier& lm, const std::string& msg);

 protected:
  void ThrowExceptionForErrorEvent(const EventLevel::event_level_t event,
                                   const std::string& msg);

 private:
  std::shared_ptr<FileWritter> file_writter_;
  std::shared_ptr<ConsoleWritter> console_writter_;
  std::shared_ptr<FormatterInterface> file_formater_;
  std::shared_ptr<FormatterInterface> console_formater_;
  std::shared_ptr<ExceptionFactory> expectation_factory_;
};

std::shared_ptr<Logger> CreateDefaultLogger(const std::string& name,
                                            const std::string& filename);

}  // namespace core::utils

#endif  // CORE_UTILS_TRACER_HPP
