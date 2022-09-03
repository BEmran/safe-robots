// Copyright (C) 2022 Bara Emran - All Rights Reserved

#ifndef CORE_UTILS_FORMATTER_HPP_
#define CORE_UTILS_FORMATTER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "core/utils/date_time.hpp"
#include "core/utils/event_level.hpp"
#include "core/utils/terminal.hpp"

namespace core::utils {

using FormatFunc =
  std::function<std::string(const LabeledModifier& lm, const std::string& msg)>;

std::string NullFormatter(const LabeledModifier& lm, const std::string& msg);

std::string TimeLabelFormatter(const LabeledModifier& lm,
                               const std::string& msg);

std::string TimeLabelModifierFormatter(const LabeledModifier& lm,
                                       const std::string& msg);

/**
 * @brief Simple class used to present message in different format (style) by
 * calling the passed formater function.
 *
 */
class Formatter {
 public:
  explicit Formatter(const FormatFunc& func = NullFormatter);

  ~Formatter() = default;

  std::string Format(const LabeledModifier& lm, const std::string& msg) const;

 private:
  FormatFunc format_func_;
};

std::shared_ptr<Formatter> CreateNullFormatter();

std::shared_ptr<Formatter> CreateTimeLabelFormatter();

std::shared_ptr<Formatter> CreateTimeLabelModifierFormatter();

}  // namespace core::utils

#endif  // CORE_UTILS_FORMATTER_HPP_
