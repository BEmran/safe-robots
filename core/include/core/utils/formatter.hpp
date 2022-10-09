// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_FORMATTER_HPP_
#define CORE_UTILS_FORMATTER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "core/utils/date_time.hpp"
#include "core/utils/labeld_modifier.hpp"
#include "core/utils/modifier.hpp"

namespace core::utils {

using FormatFunc =
  std::function<std::string(const LabeledModifier& lm, std::string_view msg)>;

/**
 * @brief Simple class used to present message in different format (style) by
 * calling the passed formater function.
 *
 */
class Formatter {
 public:
  /**
   * @brief Construct a default Formatter object with NullFormater
   *
   */
  Formatter();

  /**
   * @brief Construct a Formatter object with passed formater function
   *
   * @param func formater function
   */
  explicit Formatter(const FormatFunc& func);

  /**
   * @brief Destroy the Formatter object
   *
   */
  ~Formatter() = default;

  /**
   * @brief Apply format function on passed data
   *
   * @param lm labeled modifier
   * @param msg string message to be formatted
   * @return std::string formatted message
   */
  std::string Format(const LabeledModifier& lm, std::string_view msg) const;

 private:
  /**
   * @brief format function callback
   *
   */
  FormatFunc format_func_;
};

/**
 * @brief null formater
 *
 * @param lm labeled modifier
 * @param msg string message to be formatted
 * @return std::string formatted message
 */
std::string NullFormatter(const LabeledModifier& lm, std::string_view msg);

/**
 * @brief Format using current time and label formatter
 *
 * @param lm labeled modifier
 * @param msg string message to be formatted
 * @return std::string formatted message
 */
std::string TimeLabelFormatter(const LabeledModifier& lm, std::string_view msg);

/**
 * @brief Format using current time and time and label modifier
 *
 * @param lm labeled modifier
 * @param msg string message to be formatted
 * @return std::string formatted message
 */
std::string TimeLabelModifierFormatter(const LabeledModifier& lm,
                                       std::string_view msg);

/**
 * @brief Create a Null Formatter object
 *
 * @return Formatter a Formater object with NullFormater function as CB function
 */
Formatter CreateNullFormatter();

/**
 * @brief Create a Time Label Formatter object
 *
 * @return Formatter a Formater object with TimeLabelFormatter function as CB
 * function
 */
Formatter CreateTimeLabelFormatter();

/**
 * @brief Create a Time Label Modifier Formatter object
 *
 * @return Formatter Formater object with TimeLabelModifierFormatter function as
 * CB function
 */
Formatter CreateTimeLabelModifierFormatter();

}  // namespace core::utils

#endif  // CORE_UTILS_FORMATTER_HPP_
