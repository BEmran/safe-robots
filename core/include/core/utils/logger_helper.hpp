// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_LOGGER_HELPER_HPP_
#define CORE_UTILS_LOGGER_HELPER_HPP_

#include <cstring>  // to use strrchr
#include <memory>
#include <string>

#include "core/utils/formatter.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/writer.hpp"

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
 * @brief holds various LabeledModifier objects to be used inside the logger
 * when logging messages
 *
 */
struct LoggerLabeledModifiers {
  LabeledModifier debug;
  LabeledModifier error;
  LabeledModifier fatal;
  LabeledModifier info;
  LabeledModifier warn;

  /**
   * @brief Construct a new Node Labeled Modifiers object
   *
   * @param debug_lm debug labeled modifier
   * @param error_lm error labeled modifier
   * @param fatal_lm fatal labeled modifier
   * @param info_lm info labeled modifier
   * @param warn_lm warn labeled modifier
   */
  LoggerLabeledModifiers(const LabeledModifier& debug_lm,
                         const LabeledModifier& error_lm,
                         const LabeledModifier& fatal_lm,
                         const LabeledModifier& info_lm,
                         const LabeledModifier& warn_lm)
    : debug(debug_lm)
    , error(error_lm)
    , fatal(fatal_lm)
    , info(info_lm)
    , warn(warn_lm) {
  }

  /**
   * @brief Construct a new Node Labeled Modifiers object with default Labeled
   * Modifiers
   *
   */
  LoggerLabeledModifiers()
    : LoggerLabeledModifiers(DebugLabeledModifier(), ErrorLabeledModifier(),
                             FatalLabeledModifier(), InfoLabeledModifier(),
                             WarnLabeledModifier()) {
  }
};

}  // namespace core::utils

#endif  // CORE_UTILS_LOGGER_HELPER_HPP_
